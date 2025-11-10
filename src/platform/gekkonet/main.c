/* Copyright (c) 2025
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SDL.h>

#include <mgba-util/crc32.h>
#include <mgba-util/vfs.h>
#include <mgba/core/config.h>
#include <mgba/core/core.h>
#include <mgba/core/serialize.h>
#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/input.h>
#include <mgba/internal/gba/sio/lockstep.h>

#include <gekkonet.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#define PACKED_STATE_MAGIC 0x474E4554 /* "GNET" */
#define PACKED_STATE_VERSION 1
#define DEFAULT_STATE_SIZE (8 * 1024 * 1024)

struct Options {
	const char* romPath;
	const char* biosPath;
	char* remoteHost;
	uint16_t remotePort;
	uint16_t localPort;
	int localPlayer;
	unsigned prediction;
	unsigned localDelay;
};

struct PackedStateHeader {
	uint32_t magic;
	uint32_t version;
	uint32_t frame;
	uint32_t payload0;
	uint32_t payload1;
	uint32_t checksum;
};

struct CoreWrapper {
	struct mCore* core;
	struct mColor* videoBuffer;
	unsigned width;
	unsigned height;
};

struct SimpleLockstepUser {
	struct mLockstepUser iface;
	int preferredId;
};

struct ByteBuffer {
	uint8_t* data;
	size_t size;
	size_t capacity;
};

struct FrameState {
	int frame;
	uint8_t* data;
	size_t size;
};

struct FrameCache {
	struct FrameState* entries;
	size_t count;
	size_t capacity;
};

static void usage(const char* argv0);
static bool parse_peer(const char* value, char** hostOut, uint16_t* portOut);
static bool parse_args(int argc, char** argv, struct Options* opts);
static bool save_core_state(struct mCore* core, uint8_t** dataOut, size_t* sizeOut);
static bool load_core_state(struct mCore* core, const uint8_t* data, size_t size);
static bool pack_state(int frame, struct mCore* core0, struct mCore* core1, struct ByteBuffer* buffer,
                       uint32_t* checksum);
static bool load_packed_state(struct mCore* core0, struct mCore* core1, const uint8_t* data, size_t len);
static void update_texture(SDL_Texture* texture, const struct CoreWrapper* core);
static uint16_t update_keys_from_event(const SDL_Event* e, uint16_t current);
static bool setup_core(const struct Options* opts, const char* configName, struct CoreWrapper* wrapper);
static void teardown_core(struct CoreWrapper* wrapper);
static void configure_lockstep(struct GBASIOLockstepCoordinator* coordinator, struct GBASIOLockstepDriver drivers[2],
                               struct SimpleLockstepUser users[2], struct CoreWrapper cores[2]);
static void shutdown_lockstep(struct GBASIOLockstepCoordinator* coordinator, struct GBASIOLockstepDriver drivers[2]);
static void byte_buffer_init(struct ByteBuffer* buffer);
static void byte_buffer_deinit(struct ByteBuffer* buffer);
static bool byte_buffer_resize(struct ByteBuffer* buffer, size_t size);
static void frame_cache_init(struct FrameCache* cache);
static void frame_cache_deinit(struct FrameCache* cache);
static bool frame_cache_store(struct FrameCache* cache, int frame, const uint8_t* data, size_t size);
static void frame_cache_prune(struct FrameCache* cache, int minFrame);

static void usage(const char* argv0) {
	fprintf(stderr,
	        "Usage: %s --rom <path> --peer <host:port> --local-port <port> [options]\n"
	        "Options:\n"
	        "  --bios <path>           Override BIOS path (optional)\n"
	        "  --local-player <idx>    Local player index (0 or 1, default 0)\n"
	        "  --prediction <frames>   Input prediction window (default 6)\n"
	        "  --local-delay <frames>  Frames of local input delay (default 0)\n",
	        argv0);
}

static bool parse_peer(const char* value, char** hostOut, uint16_t* portOut) {
	const char* colon = strchr(value, ':');
	char* host;
	char* endptr;
	long port;
	size_t hostLen;

	if (!colon) {
		return false;
	}
	hostLen = (size_t) (colon - value);
	host = (char*) malloc(hostLen + 1);
	if (!host) {
		return false;
	}
	memcpy(host, value, hostLen);
	host[hostLen] = '\0';
	port = strtol(colon + 1, &endptr, 10);
	if (!endptr || *endptr != '\0' || port <= 0 || port > 65535) {
		free(host);
		return false;
	}
	*hostOut = host;
	*portOut = (uint16_t) port;
	return true;
}

static bool parse_args(int argc, char** argv, struct Options* opts) {
	int i;

	memset(opts, 0, sizeof(*opts));
	opts->prediction = 6;
	for (i = 1; i < argc; ++i) {
		const char* arg = argv[i];
		if (strcmp(arg, "--rom") == 0 && i + 1 < argc) {
			opts->romPath = argv[++i];
		} else if (strcmp(arg, "--bios") == 0 && i + 1 < argc) {
			opts->biosPath = argv[++i];
		} else if (strcmp(arg, "--peer") == 0 && i + 1 < argc) {
			if (!parse_peer(argv[++i], &opts->remoteHost, &opts->remotePort)) {
				goto error;
			}
		} else if (strcmp(arg, "--local-port") == 0 && i + 1 < argc) {
			long port = strtol(argv[++i], NULL, 10);
			if (port <= 0 || port > 65535) {
				goto error;
			}
			opts->localPort = (uint16_t) port;
		} else if (strcmp(arg, "--local-player") == 0 && i + 1 < argc) {
			long idx = strtol(argv[++i], NULL, 10);
			if (idx < 0 || idx > 1) {
				goto error;
			}
			opts->localPlayer = (int) idx;
		} else if (strcmp(arg, "--prediction") == 0 && i + 1 < argc) {
			long frames = strtol(argv[++i], NULL, 10);
			if (frames < 0 || frames > 12) {
				goto error;
			}
			opts->prediction = (unsigned) frames;
		} else if (strcmp(arg, "--local-delay") == 0 && i + 1 < argc) {
			long frames = strtol(argv[++i], NULL, 10);
			if (frames < 0 || frames > 12) {
				goto error;
			}
			opts->localDelay = (unsigned) frames;
		} else if (strcmp(arg, "--help") == 0) {
			goto error;
		} else {
			fprintf(stderr, "Unknown argument: %s\n", arg);
			goto error;
		}
	}
	if (!opts->romPath || !opts->remoteHost || !opts->remotePort || !opts->localPort) {
		goto error;
	}
	return true;

error:
	free(opts->remoteHost);
	opts->remoteHost = NULL;
	return false;
}

static bool save_core_state(struct mCore* core, uint8_t** dataOut, size_t* sizeOut) {
	VFile* vf = VFileMemChunk(NULL, 0);
	size_t size;
	uint8_t* data;

	if (!vf) {
		return false;
	}
	if (!mCoreSaveStateNamed(core, vf, SAVESTATE_SAVEDATA | SAVESTATE_RTC | SAVESTATE_METADATA)) {
		vf->close(vf);
		return false;
	}
	size = vf->size(vf);
	data = (uint8_t*) malloc(size);
	if (!data) {
		vf->close(vf);
		return false;
	}
	vf->seek(vf, 0, SEEK_SET);
	if (vf->read(vf, data, size) != (ssize_t) size) {
		free(data);
		vf->close(vf);
		return false;
	}
	vf->close(vf);
	*dataOut = data;
	*sizeOut = size;
	return true;
}

static bool load_core_state(struct mCore* core, const uint8_t* data, size_t size) {
	VFile* vf = VFileMemChunk(NULL, 0);
	bool ok = false;

	if (!vf) {
		return false;
	}
	if (vf->write(vf, data, size) != (ssize_t) size) {
		vf->close(vf);
		return false;
	}
	vf->seek(vf, 0, SEEK_SET);
	ok = mCoreLoadStateNamed(core, vf, SAVESTATE_SAVEDATA | SAVESTATE_RTC | SAVESTATE_METADATA);
	vf->close(vf);
	return ok;
}

static void byte_buffer_init(struct ByteBuffer* buffer) {
	buffer->data = NULL;
	buffer->size = 0;
	buffer->capacity = 0;
}

static void byte_buffer_deinit(struct ByteBuffer* buffer) {
	free(buffer->data);
	buffer->data = NULL;
	buffer->size = 0;
	buffer->capacity = 0;
}

static bool byte_buffer_resize(struct ByteBuffer* buffer, size_t size) {
	uint8_t* resized;
	if (size > buffer->capacity) {
		resized = (uint8_t*) realloc(buffer->data, size);
		if (!resized) {
			return false;
		}
		buffer->data = resized;
		buffer->capacity = size;
	}
	buffer->size = size;
	return true;
}

static bool pack_state(int frame, struct mCore* core0, struct mCore* core1, struct ByteBuffer* buffer,
                       uint32_t* checksum) {
	struct PackedStateHeader header;
	uint8_t* state0 = NULL;
	uint8_t* state1 = NULL;
	size_t size0 = 0;
	size_t size1 = 0;
	size_t total;

	if (!save_core_state(core0, &state0, &size0) || !save_core_state(core1, &state1, &size1)) {
		free(state0);
		free(state1);
		return false;
	}
	total = sizeof(header) + size0 + size1;
	if (!byte_buffer_resize(buffer, total)) {
		free(state0);
		free(state1);
		return false;
	}
	header.magic = PACKED_STATE_MAGIC;
	header.version = PACKED_STATE_VERSION;
	header.frame = (uint32_t) frame;
	header.payload0 = (uint32_t) size0;
	header.payload1 = (uint32_t) size1;
	header.checksum = 0;
	memcpy(buffer->data, &header, sizeof(header));
	memcpy(buffer->data + sizeof(header), state0, size0);
	memcpy(buffer->data + sizeof(header) + size0, state1, size1);
	*checksum = crc32(0, buffer->data, buffer->size);
	header.checksum = *checksum;
	memcpy(buffer->data, &header, sizeof(header));
	free(state0);
	free(state1);
	return true;
}

static bool unpack_state(const uint8_t* data, size_t len, int* frameOut, uint8_t** out0, size_t* size0, uint8_t** out1,
                         size_t* size1) {
	const struct PackedStateHeader* header;
	const uint8_t* payload;
	size_t expected;

	if (len < sizeof(struct PackedStateHeader)) {
		return false;
	}
	header = (const struct PackedStateHeader*) data;
	if (header->magic != PACKED_STATE_MAGIC || header->version != PACKED_STATE_VERSION) {
		return false;
	}
	expected = sizeof(struct PackedStateHeader) + header->payload0 + header->payload1;
	if (expected > len) {
		return false;
	}
	payload = data + sizeof(struct PackedStateHeader);
	*out0 = (uint8_t*) malloc(header->payload0);
	*out1 = (uint8_t*) malloc(header->payload1);
	if ((!header->payload0 || *out0) && (!header->payload1 || *out1)) {
		if (header->payload0) {
			memcpy(*out0, payload, header->payload0);
		}
		if (header->payload1) {
			memcpy(*out1, payload + header->payload0, header->payload1);
		}
		*size0 = header->payload0;
		*size1 = header->payload1;
		*frameOut = (int) header->frame;
		return true;
	}
	free(*out0);
	free(*out1);
	*out0 = NULL;
	*out1 = NULL;
	return false;
}

static bool load_packed_state(struct mCore* core0, struct mCore* core1, const uint8_t* data, size_t len) {
	uint8_t* tmp0 = NULL;
	uint8_t* tmp1 = NULL;
	int frame = 0;
	size_t size0 = 0;
	size_t size1 = 0;
	bool ok = false;

	if (!unpack_state(data, len, &frame, &tmp0, &size0, &tmp1, &size1)) {
		return false;
	}
	ok = load_core_state(core0, tmp0, size0) && load_core_state(core1, tmp1, size1);
	free(tmp0);
	free(tmp1);
	return ok;
}

static void update_texture(SDL_Texture* texture, const struct CoreWrapper* core) {
	void* pixels;
	int pitch;
	unsigned y;
	size_t rowBytes;
	uint8_t* dst;

	if (!texture || !core->videoBuffer) {
		return;
	}
	if (SDL_LockTexture(texture, NULL, &pixels, &pitch) != 0) {
		fprintf(stderr, "Failed to lock SDL texture: %s\n", SDL_GetError());
		return;
	}
	rowBytes = core->width * sizeof(mColor);
	dst = (uint8_t*) pixels;
	for (y = 0; y < core->height; ++y) {
		memcpy(dst + y * pitch, (const uint8_t*) core->videoBuffer + y * rowBytes, rowBytes);
	}
	SDL_UnlockTexture(texture);
}

static uint16_t update_keys_from_event(const SDL_Event* e, uint16_t current) {
	int pressed = 0;

	if (e->type == SDL_KEYDOWN && !e->key.repeat) {
		pressed = 1;
	} else if (e->type == SDL_KEYUP) {
		pressed = -1;
	}
	if (!pressed) {
		return current;
	}
	switch (e->key.keysym.sym) {
	case SDLK_x:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_A);
		} else {
			current &= ~(1u << GBA_KEY_A);
		}
		break;
	case SDLK_z:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_B);
		} else {
			current &= ~(1u << GBA_KEY_B);
		}
		break;
	case SDLK_BACKSPACE:
	case SDLK_RSHIFT:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_SELECT);
		} else {
			current &= ~(1u << GBA_KEY_SELECT);
		}
		break;
	case SDLK_RETURN:
	case SDLK_RETURN2:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_START);
		} else {
			current &= ~(1u << GBA_KEY_START);
		}
		break;
	case SDLK_RIGHT:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_RIGHT);
		} else {
			current &= ~(1u << GBA_KEY_RIGHT);
		}
		break;
	case SDLK_LEFT:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_LEFT);
		} else {
			current &= ~(1u << GBA_KEY_LEFT);
		}
		break;
	case SDLK_UP:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_UP);
		} else {
			current &= ~(1u << GBA_KEY_UP);
		}
		break;
	case SDLK_DOWN:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_DOWN);
		} else {
			current &= ~(1u << GBA_KEY_DOWN);
		}
		break;
	case SDLK_a:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_L);
		} else {
			current &= ~(1u << GBA_KEY_L);
		}
		break;
	case SDLK_s:
		if (pressed > 0) {
			current |= (1u << GBA_KEY_R);
		} else {
			current &= ~(1u << GBA_KEY_R);
		}
		break;
	default:
		break;
	}
	return current;
}

static bool setup_core(const struct Options* opts, const char* configName, struct CoreWrapper* wrapper) {
	struct mCore* core = mCoreFind(opts->romPath);

	memset(wrapper, 0, sizeof(*wrapper));
	if (!core) {
		fprintf(stderr, "Failed to find core for ROM: %s\n", opts->romPath);
		return false;
	}
	if (!core->init(core)) {
		fprintf(stderr, "Failed to initialize core\n");
		return false;
	}
	core->baseVideoSize(core, &wrapper->width, &wrapper->height);
	wrapper->videoBuffer = (mColor*) malloc(wrapper->width * wrapper->height * sizeof(mColor));
	if (!wrapper->videoBuffer) {
		fprintf(stderr, "Failed to allocate video buffer\n");
		core->deinit(core);
		return false;
	}
	core->setVideoBuffer(core, wrapper->videoBuffer, wrapper->width);
	mCoreInitConfig(core, configName);
	mCoreConfigSetDefaultValue(&core->config, "idleOptimization", "remove");
	mCoreConfigSetDefaultIntValue(&core->config, "logToStdout", true);
	mCoreLoadConfig(core);
	if (!mCoreLoadFile(core, opts->romPath)) {
		fprintf(stderr, "Failed to load ROM\n");
		mCoreConfigDeinit(&core->config);
		core->deinit(core);
		free(wrapper->videoBuffer);
		return false;
	}
	if (opts->biosPath) {
		VFile* bios = VFileOpen(opts->biosPath, O_RDONLY);
		if (bios) {
			core->loadBIOS(core, bios, 0);
			bios->close(bios);
		}
	}
	core->reset(core);
	wrapper->core = core;
	return true;
}

static void teardown_core(struct CoreWrapper* wrapper) {
	if (!wrapper->core) {
		return;
	}
	wrapper->core->unloadROM(wrapper->core);
	mCoreConfigDeinit(&wrapper->core->config);
	wrapper->core->deinit(wrapper->core);
	free(wrapper->videoBuffer);
	wrapper->videoBuffer = NULL;
	wrapper->core = NULL;
}

static int simple_lockstep_requested_id(struct mLockstepUser* ctx) {
	struct SimpleLockstepUser* user = (struct SimpleLockstepUser*) ctx;
	return user->preferredId;
}

static void configure_lockstep(struct GBASIOLockstepCoordinator* coordinator, struct GBASIOLockstepDriver drivers[2],
                               struct SimpleLockstepUser users[2], struct CoreWrapper cores[2]) {
	int i;

	GBASIOLockstepCoordinatorInit(coordinator);
	for (i = 0; i < 2; ++i) {
		memset(&users[i], 0, sizeof(users[i]));
		memset(&drivers[i], 0, sizeof(drivers[i]));
		users[i].preferredId = i;
		users[i].iface.requestedId = simple_lockstep_requested_id;
		GBASIOLockstepDriverCreate(&drivers[i], &users[i].iface);
		drivers[i].coordinator = coordinator;
		GBASIOLockstepCoordinatorAttach(coordinator, &drivers[i]);
		cores[i].core->setPeripheral(cores[i].core, mPERIPH_GBA_LINK_PORT, &drivers[i].d);
	}
}

static void shutdown_lockstep(struct GBASIOLockstepCoordinator* coordinator, struct GBASIOLockstepDriver drivers[2]) {
	int i;
	for (i = 0; i < 2; ++i) {
		GBASIOLockstepCoordinatorDetach(coordinator, &drivers[i]);
	}
	GBASIOLockstepCoordinatorDeinit(coordinator);
}

static void frame_cache_init(struct FrameCache* cache) {
	cache->entries = NULL;
	cache->count = 0;
	cache->capacity = 0;
}

static void frame_cache_deinit(struct FrameCache* cache) {
	size_t i;
	for (i = 0; i < cache->count; ++i) {
		free(cache->entries[i].data);
	}
	free(cache->entries);
	cache->entries = NULL;
	cache->count = 0;
	cache->capacity = 0;
}

static bool frame_cache_store(struct FrameCache* cache, int frame, const uint8_t* data, size_t size) {
	struct FrameState* entry;
	uint8_t* copy;
	size_t i;

	for (i = 0; i < cache->count; ++i) {
		if (cache->entries[i].frame == frame) {
			free(cache->entries[i].data);
			cache->entries[i].data = NULL;
			cache->entries[i].size = 0;
			cache->entries[i].frame = frame;
			break;
		}
	}
	if (i == cache->count) {
		if (cache->count == cache->capacity) {
			size_t newCapacity = cache->capacity ? cache->capacity * 2 : 8;
			struct FrameState* grown =
			    (struct FrameState*) realloc(cache->entries, newCapacity * sizeof(struct FrameState));
			if (!grown) {
				return false;
			}
			cache->entries = grown;
			cache->capacity = newCapacity;
		}
		i = cache->count++;
	}
	entry = &cache->entries[i];
	if (size) {
		copy = (uint8_t*) malloc(size);
		if (!copy) {
			return false;
		}
		memcpy(copy, data, size);
	} else {
		copy = NULL;
	}
	entry->frame = frame;
	entry->data = copy;
	entry->size = size;
	return true;
}

static void frame_cache_prune(struct FrameCache* cache, int minFrame) {
	size_t i = 0;
	while (i < cache->count) {
		if (cache->entries[i].frame + 96 < minFrame) {
			free(cache->entries[i].data);
			cache->entries[i] = cache->entries[cache->count - 1];
			--cache->count;
		} else {
			++i;
		}
	}
}

int main(int argc, char** argv) {
	struct Options opts;
	struct CoreWrapper cores[2];
	SDL_Window* window = NULL;
	SDL_Renderer* renderer = NULL;
	SDL_Texture* texture = NULL;
	struct GBASIOLockstepCoordinator coordinator;
	struct GBASIOLockstepDriver drivers[2];
	struct SimpleLockstepUser users[2];
	GekkoSession* session = NULL;
	GekkoNetAdapter* adapter = NULL;
	int localHandle;
	int remoteHandle;
	GekkoConfig config;
	struct ByteBuffer packBuffer;
	struct FrameCache stateCache;
	bool running = true;
	uint16_t localKeys = 0;
	int currentFrame = 0;
	Uint32 lastPresent;
	const double targetFrameMs = 1000.0 / 60.0;
	int i;

#if defined(_WIN32)
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		fprintf(stderr, "Failed to initialize WinSock\n");
		return EXIT_FAILURE;
	}
#endif

	if (!parse_args(argc, argv, &opts)) {
		usage(argv[0]);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
		fprintf(stderr, "Failed to initialize SDL: %s\n", SDL_GetError());
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	for (i = 0; i < 2; ++i) {
		cores[i].core = NULL;
		cores[i].videoBuffer = NULL;
	}
	if (!setup_core(&opts, "gekkonet-0", &cores[0]) || !setup_core(&opts, "gekkonet-1", &cores[1])) {
		teardown_core(&cores[0]);
		teardown_core(&cores[1]);
		SDL_Quit();
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	window = SDL_CreateWindow("mGBA GekkoNet", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
	                          (int) (cores[0].width * 2), (int) (cores[0].height * 2), SDL_WINDOW_RESIZABLE);
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (renderer) {
		texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
		                            (int) cores[0].width, (int) cores[0].height);
	}
	if (!window || !renderer || !texture) {
		fprintf(stderr, "Failed to create SDL resources: %s\n", SDL_GetError());
		if (texture) {
			SDL_DestroyTexture(texture);
		}
		if (renderer) {
			SDL_DestroyRenderer(renderer);
		}
		if (window) {
			SDL_DestroyWindow(window);
		}
		teardown_core(&cores[0]);
		teardown_core(&cores[1]);
		SDL_Quit();
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	configure_lockstep(&coordinator, drivers, users, cores);
	if (!gekko_create(&session)) {
		fprintf(stderr, "Failed to create GekkoNet session\n");
		shutdown_lockstep(&coordinator, drivers);
		teardown_core(&cores[0]);
		teardown_core(&cores[1]);
		SDL_DestroyTexture(texture);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	memset(&config, 0, sizeof(config));
	config.num_players = 2;
	config.max_spectators = 0;
	config.input_prediction_window = (unsigned char) opts.prediction;
	config.spectator_delay = 0;
	config.input_size = sizeof(uint16_t);
	config.state_size = DEFAULT_STATE_SIZE;
	config.limited_saving = false;
	config.post_sync_joining = false;
	config.desync_detection = true;

	adapter = gekko_default_adapter(opts.localPort);
	if (!adapter) {
		fprintf(stderr, "Failed to create GekkoNet adapter\n");
		gekko_destroy(session);
		shutdown_lockstep(&coordinator, drivers);
		teardown_core(&cores[0]);
		teardown_core(&cores[1]);
		SDL_DestroyTexture(texture);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}
	gekko_net_adapter_set(session, adapter);

	localHandle = gekko_add_actor(session, LocalPlayer, NULL);

	if (localHandle < 0) {
		fprintf(stderr, "Failed to register local actor\n");
		gekko_destroy(session);
		shutdown_lockstep(&coordinator, drivers);
		teardown_core(&cores[0]);
		teardown_core(&cores[1]);
		SDL_DestroyTexture(texture);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	{
		sockaddr_in remoteAddr;
		GekkoNetAddress addr;
		memset(&remoteAddr, 0, sizeof(remoteAddr));
		remoteAddr.sin_family = AF_INET;
		remoteAddr.sin_port = htons(opts.remotePort);
		if (inet_pton(AF_INET, opts.remoteHost, &remoteAddr.sin_addr) != 1) {
			fprintf(stderr, "Invalid remote host: %s\n", opts.remoteHost);
			gekko_destroy(session);
			shutdown_lockstep(&coordinator, drivers);
			teardown_core(&cores[0]);
			teardown_core(&cores[1]);
			SDL_DestroyTexture(texture);
			SDL_DestroyRenderer(renderer);
			SDL_DestroyWindow(window);
			SDL_Quit();
			free(opts.remoteHost);
#if defined(_WIN32)
			WSACleanup();
#endif
			return EXIT_FAILURE;
		}
		memset(&addr, 0, sizeof(addr));
		addr.data = &remoteAddr;
		addr.size = sizeof(remoteAddr);
		remoteHandle = gekko_add_actor(session, RemotePlayer, &addr);
	}
	if (remoteHandle < 0) {
		fprintf(stderr, "Failed to register remote actor\n");
		gekko_destroy(session);
		shutdown_lockstep(&coordinator, drivers);
		teardown_core(&cores[0]);
		teardown_core(&cores[1]);
		SDL_DestroyTexture(texture);
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		SDL_Quit();
		free(opts.remoteHost);
#if defined(_WIN32)
		WSACleanup();
#endif
		return EXIT_FAILURE;
	}

	gekko_start(session, &config);
	if (opts.localDelay) {
		gekko_set_local_delay(session, localHandle, (unsigned char) opts.localDelay);
	}

	byte_buffer_init(&packBuffer);
	frame_cache_init(&stateCache);
	lastPresent = SDL_GetTicks();

	while (running) {
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				running = false;
			} else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
				running = false;
			} else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
				localKeys = update_keys_from_event(&event, localKeys);
			}
		}

		gekko_add_local_input(session, localHandle, &localKeys);
		gekko_network_poll(session);

		{
			int sessionEventsCount = 0;
			GekkoSessionEvent** sessionEvents = gekko_session_events(session, &sessionEventsCount);
			for (i = 0; i < sessionEventsCount; ++i) {
				GekkoSessionEvent* sev = sessionEvents[i];
				if (!sev) {
					continue;
				}
				switch (sev->type) {
				case PlayerConnected:
					printf("Player %d connected\n", sev->data.connected.handle);
					break;
				case PlayerDisconnected:
					printf("Player %d disconnected\n", sev->data.disconnected.handle);
					break;
				case DesyncDetected:
					fprintf(stderr, "Desync detected at frame %d\n", sev->data.desynced.frame);
					break;
				default:
					break;
				}
			}
		}

		{
			int eventCount = 0;
			GekkoGameEvent** events = gekko_update_session(session, &eventCount);
			for (i = 0; i < eventCount; ++i) {
				GekkoGameEvent* ev = events[i];
				if (!ev) {
					continue;
				}
				switch (ev->type) {
				case SaveEvent: {
					uint32_t checksum = 0;
					if (!pack_state(ev->data.save.frame, cores[0].core, cores[1].core, &packBuffer, &checksum)) {
						fprintf(stderr, "Failed to serialize state for frame %d\n", ev->data.save.frame);
						break;
					}
					if (packBuffer.size > config.state_size) {
						fprintf(stderr, "Serialized state exceeds configured size limit\n");
						break;
					}
					memcpy(ev->data.save.state, packBuffer.data, packBuffer.size);
					*ev->data.save.state_len = (unsigned int) packBuffer.size;
					*ev->data.save.checksum = checksum;
					if (!frame_cache_store(&stateCache, ev->data.save.frame, packBuffer.data, packBuffer.size)) {
						fprintf(stderr, "Failed to cache rollback state for frame %d\n", ev->data.save.frame);
					}
					break;
				}
				case LoadEvent:
					if (!load_packed_state(cores[0].core, cores[1].core, ev->data.load.state,
					                       ev->data.load.state_len)) {
						fprintf(stderr, "Failed to load rollback state at frame %d\n", ev->data.load.frame);
					}
					currentFrame = ev->data.load.frame;
					break;
				case AdvanceEvent: {
					const uint16_t* inputs = (const uint16_t*) ev->data.adv.inputs;
					uint16_t mappedInputs[2];
					if (ev->data.adv.input_len < sizeof(uint16_t) * 2) {
						break;
					}
					mappedInputs[opts.localPlayer] = inputs[0];
					mappedInputs[1 - opts.localPlayer] = inputs[1];
					cores[0].core->setKeys(cores[0].core, mappedInputs[0]);
					cores[1].core->setKeys(cores[1].core, mappedInputs[1]);
					cores[0].core->runFrame(cores[0].core);
					cores[1].core->runFrame(cores[1].core);
					currentFrame = ev->data.adv.frame;
					{
						Uint32 now = SDL_GetTicks();
						double elapsed = (double) (now - lastPresent);
						if (elapsed < targetFrameMs) {
							SDL_Delay((Uint32) (targetFrameMs - elapsed));
						}
					}
					update_texture(texture, &cores[0]);
					SDL_RenderClear(renderer);
					SDL_RenderCopy(renderer, texture, NULL, NULL);
					SDL_RenderPresent(renderer);
					lastPresent = SDL_GetTicks();
					frame_cache_prune(&stateCache, ev->data.adv.frame);
					break;
				}
				default:
					break;
				}
			}
		}
	}

	byte_buffer_deinit(&packBuffer);
	frame_cache_deinit(&stateCache);
	gekko_destroy(session);
	shutdown_lockstep(&coordinator, drivers);
	teardown_core(&cores[0]);
	teardown_core(&cores[1]);
	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
	free(opts.remoteHost);
#if defined(_WIN32)
	WSACleanup();
#endif
	(void) currentFrame;
	return EXIT_SUCCESS;
}
