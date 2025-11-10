#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <SDL.h>

extern "C" {
#include <mgba/core/config.h>
#include <mgba/core/core.h>
#include <mgba/core/serialize.h>
#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/input.h>
#include <mgba/internal/gba/sio/lockstep.h>
#include <mgba-util/crc32.h>
#include <mgba-util/vfs.h>
}

#include <gekkonet.h>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

namespace {

constexpr uint32_t kPackedStateMagic = 0x474E4554; // "GNET"
constexpr uint32_t kPackedStateVersion = 1;
constexpr uint32_t kDefaultStateSize = 8 * 1024 * 1024; // 8 MiB

struct Options {
        std::string romPath;
        std::string biosPath;
        std::string remoteHost;
        uint16_t remotePort = 0;
        uint16_t localPort = 0;
        int localPlayer = 0;
        unsigned prediction = 6;
        unsigned localDelay = 0;
};

struct PackedStateHeader {
        uint32_t magic;
        uint32_t version;
        uint32_t frame;
        uint32_t payload0;
        uint32_t payload1;
        uint32_t checksum;
};

struct FrameState {
        std::vector<uint8_t> data;
};

struct SimpleLockstepUser {
        mLockstepUser iface{};
        int preferredId = 0;
};

struct CoreWrapper {
        mCore* core = nullptr;
        std::vector<mColor> videoBuffer;
        unsigned width = 0;
        unsigned height = 0;
};

void usage(const char* argv0) {
        std::cerr << "Usage: " << argv0 << " --rom <path> --peer <host:port> --local-port <port> [options]\n";
        std::cerr << "Options:\n"
                  << "  --bios <path>           Override BIOS path (optional)\n"
                  << "  --local-player <idx>    Local player index (0 or 1, default 0)\n"
                  << "  --prediction <frames>   Input prediction window (default 6)\n"
                  << "  --local-delay <frames>  Frames of local input delay (default 0)\n";
}

bool parsePeer(const std::string& value, std::string& hostOut, uint16_t& portOut) {
        auto colon = value.find(':');
        if (colon == std::string::npos) {
                return false;
        }
        hostOut = value.substr(0, colon);
        std::string portStr = value.substr(colon + 1);
        char* end = nullptr;
        long port = std::strtol(portStr.c_str(), &end, 10);
        if (!end || *end != '\0' || port <= 0 || port > 65535) {
                return false;
        }
        portOut = static_cast<uint16_t>(port);
        return true;
}

bool parseArgs(int argc, char** argv, Options& opts) {
        for (int i = 1; i < argc; ++i) {
                std::string arg(argv[i]);
                if (arg == "--rom" && i + 1 < argc) {
                        opts.romPath = argv[++i];
                } else if (arg == "--bios" && i + 1 < argc) {
                        opts.biosPath = argv[++i];
                } else if (arg == "--peer" && i + 1 < argc) {
                        if (!parsePeer(argv[++i], opts.remoteHost, opts.remotePort)) {
                                return false;
                        }
                } else if (arg == "--local-port" && i + 1 < argc) {
                        long port = std::strtol(argv[++i], nullptr, 10);
                        if (port <= 0 || port > 65535) {
                                return false;
                        }
                        opts.localPort = static_cast<uint16_t>(port);
                } else if (arg == "--local-player" && i + 1 < argc) {
                        long idx = std::strtol(argv[++i], nullptr, 10);
                        if (idx < 0 || idx > 1) {
                                return false;
                        }
                        opts.localPlayer = static_cast<int>(idx);
                } else if (arg == "--prediction" && i + 1 < argc) {
                        long frames = std::strtol(argv[++i], nullptr, 10);
                        if (frames < 0 || frames > 12) {
                                return false;
                        }
                        opts.prediction = static_cast<unsigned>(frames);
                } else if (arg == "--local-delay" && i + 1 < argc) {
                        long frames = std::strtol(argv[++i], nullptr, 10);
                        if (frames < 0 || frames > 12) {
                                return false;
                        }
                        opts.localDelay = static_cast<unsigned>(frames);
                } else if (arg == "--help") {
                        return false;
                } else {
                        std::cerr << "Unknown argument: " << arg << "\n";
                        return false;
                }
        }
        if (opts.romPath.empty() || opts.remoteHost.empty() || opts.remotePort == 0 || opts.localPort == 0) {
                return false;
        }
        return true;
}

bool saveCoreState(mCore* core, std::vector<uint8_t>& out) {
        VFile* vf = VFileMemChunk(nullptr, 0);
        if (!vf) {
                                return false;
        }
        bool ok = mCoreSaveStateNamed(core, vf, SAVESTATE_SAVEDATA | SAVESTATE_RTC | SAVESTATE_METADATA);
        if (!ok) {
                vf->close(vf);
                return false;
        }
        size_t size = vf->size(vf);
        out.resize(size);
        vf->seek(vf, 0, SEEK_SET);
        if (vf->read(vf, out.data(), size) != static_cast<ssize_t>(size)) {
                vf->close(vf);
                return false;
        }
        vf->close(vf);
        return true;
}

bool loadCoreState(mCore* core, const uint8_t* data, size_t size) {
        VFile* vf = VFileMemChunk(nullptr, 0);
        if (!vf) {
                return false;
        }
        if (vf->write(vf, data, size) != static_cast<ssize_t>(size)) {
                vf->close(vf);
                return false;
        }
        vf->seek(vf, 0, SEEK_SET);
        bool ok = mCoreLoadStateNamed(core, vf, SAVESTATE_SAVEDATA | SAVESTATE_RTC | SAVESTATE_METADATA);
        vf->close(vf);
        return ok;
}

bool packState(int frame, mCore* core0, mCore* core1, std::vector<uint8_t>& buffer, uint32_t& checksum) {
        std::vector<uint8_t> state0;
        std::vector<uint8_t> state1;
        if (!saveCoreState(core0, state0) || !saveCoreState(core1, state1)) {
                return false;
        }
        PackedStateHeader header{};
        header.magic = kPackedStateMagic;
        header.version = kPackedStateVersion;
        header.frame = static_cast<uint32_t>(frame);
        header.payload0 = static_cast<uint32_t>(state0.size());
        header.payload1 = static_cast<uint32_t>(state1.size());
        size_t total = sizeof(header) + state0.size() + state1.size();
        buffer.resize(total);
        std::memcpy(buffer.data(), &header, sizeof(header));
        std::memcpy(buffer.data() + sizeof(header), state0.data(), state0.size());
        std::memcpy(buffer.data() + sizeof(header) + state0.size(), state1.data(), state1.size());
        checksum = crc32(0, buffer.data(), buffer.size());
        header.checksum = checksum;
        std::memcpy(buffer.data(), &header, sizeof(header));
        return true;
}

bool unpackState(const uint8_t* data, size_t len, int& frameOut, std::vector<uint8_t>& tmp0, std::vector<uint8_t>& tmp1) {
        if (len < sizeof(PackedStateHeader)) {
                return false;
        }
        const auto* header = reinterpret_cast<const PackedStateHeader*>(data);
        if (header->magic != kPackedStateMagic || header->version != kPackedStateVersion) {
                return false;
        }
        size_t expected = sizeof(PackedStateHeader) + header->payload0 + header->payload1;
        if (expected > len) {
                return false;
        }
        frameOut = static_cast<int>(header->frame);
        const uint8_t* payload = data + sizeof(PackedStateHeader);
        tmp0.assign(payload, payload + header->payload0);
        tmp1.assign(payload + header->payload0, payload + header->payload0 + header->payload1);
        return true;
}

bool loadPackedState(mCore* core0, mCore* core1, const uint8_t* data, size_t len) {
        std::vector<uint8_t> tmp0;
        std::vector<uint8_t> tmp1;
        int frame = 0;
        if (!unpackState(data, len, frame, tmp0, tmp1)) {
                return false;
        }
        if (!loadCoreState(core0, tmp0.data(), tmp0.size())) {
                return false;
        }
        if (!loadCoreState(core1, tmp1.data(), tmp1.size())) {
                return false;
        }
        return true;
}

void updateTexture(SDL_Texture* texture, const CoreWrapper& core) {
        if (!texture || core.videoBuffer.empty()) {
                return;
        }
        void* pixels = nullptr;
        int pitch = 0;
        if (SDL_LockTexture(texture, nullptr, &pixels, &pitch) != 0) {
                std::cerr << "Failed to lock SDL texture: " << SDL_GetError() << "\n";
                return;
        }
        const uint8_t* src = reinterpret_cast<const uint8_t*>(core.videoBuffer.data());
        size_t rowBytes = core.width * sizeof(mColor);
        uint8_t* dst = static_cast<uint8_t*>(pixels);
        for (unsigned y = 0; y < core.height; ++y) {
                std::memcpy(dst + y * pitch, src + y * rowBytes, rowBytes);
        }
        SDL_UnlockTexture(texture);
}

uint16_t updateKeysFromEvent(const SDL_Event& e, uint16_t current) {
        auto apply = [&](bool pressed) {
                switch (e.key.keysym.sym) {
                case SDLK_x:
                        if (pressed) current |= (1u << GBA_KEY_A); else current &= ~(1u << GBA_KEY_A);
                        break;
                case SDLK_z:
                        if (pressed) current |= (1u << GBA_KEY_B); else current &= ~(1u << GBA_KEY_B);
                        break;
                case SDLK_BACKSPACE:
                case SDLK_RSHIFT:
                        if (pressed) current |= (1u << GBA_KEY_SELECT); else current &= ~(1u << GBA_KEY_SELECT);
                        break;
                case SDLK_RETURN:
                case SDLK_RETURN2:
                        if (pressed) current |= (1u << GBA_KEY_START); else current &= ~(1u << GBA_KEY_START);
                        break;
                case SDLK_RIGHT:
                        if (pressed) current |= (1u << GBA_KEY_RIGHT); else current &= ~(1u << GBA_KEY_RIGHT);
                        break;
                case SDLK_LEFT:
                        if (pressed) current |= (1u << GBA_KEY_LEFT); else current &= ~(1u << GBA_KEY_LEFT);
                        break;
                case SDLK_UP:
                        if (pressed) current |= (1u << GBA_KEY_UP); else current &= ~(1u << GBA_KEY_UP);
                        break;
                case SDLK_DOWN:
                        if (pressed) current |= (1u << GBA_KEY_DOWN); else current &= ~(1u << GBA_KEY_DOWN);
                        break;
                case SDLK_a:
                        if (pressed) current |= (1u << GBA_KEY_L); else current &= ~(1u << GBA_KEY_L);
                        break;
                case SDLK_s:
                        if (pressed) current |= (1u << GBA_KEY_R); else current &= ~(1u << GBA_KEY_R);
                        break;
                default:
                        break;
                }
        };
        if (e.type == SDL_KEYDOWN && !e.key.repeat) {
                apply(true);
        } else if (e.type == SDL_KEYUP) {
                apply(false);
        }
        return current;
}

bool setupCore(const Options& opts, const std::string& configName, CoreWrapper& wrapper) {
        wrapper.core = mCoreFind(opts.romPath.c_str());
        if (!wrapper.core) {
                std::cerr << "Failed to find core for ROM: " << opts.romPath << "\n";
                return false;
        }
        if (!wrapper.core->init(wrapper.core)) {
                std::cerr << "Failed to initialize core\n";
                return false;
        }
        wrapper.core->baseVideoSize(wrapper.core, &wrapper.width, &wrapper.height);
        wrapper.videoBuffer.resize(wrapper.width * wrapper.height);
        wrapper.core->setVideoBuffer(wrapper.core, wrapper.videoBuffer.data(), wrapper.width);
        mCoreInitConfig(wrapper.core, configName.c_str());
        mCoreConfigSetDefaultValue(&wrapper.core->config, "idleOptimization", "remove");
        mCoreConfigSetDefaultIntValue(&wrapper.core->config, "logToStdout", true);
        mCoreLoadConfig(wrapper.core);
        if (!mCoreLoadFile(wrapper.core, opts.romPath.c_str())) {
                std::cerr << "Failed to load ROM" << std::endl;
                return false;
        }
        if (!opts.biosPath.empty()) {
                VFile* bios = VFileOpen(opts.biosPath.c_str(), O_RDONLY);
                if (bios) {
                        wrapper.core->loadBIOS(wrapper.core, bios, 0);
                        bios->close(bios);
                }
        }
        wrapper.core->reset(wrapper.core);
        return true;
}

void teardownCore(CoreWrapper& wrapper) {
        if (!wrapper.core) {
                return;
        }
        wrapper.core->unloadROM(wrapper.core);
        mCoreConfigDeinit(&wrapper.core->config);
        wrapper.core->deinit(wrapper.core);
        wrapper.core = nullptr;
}

void configureLockstep(GBASIOLockstepCoordinator& coordinator, std::array<GBASIOLockstepDriver, 2>& drivers,
        std::array<SimpleLockstepUser, 2>& users, std::array<CoreWrapper, 2>& cores) {
        GBASIOLockstepCoordinatorInit(&coordinator);
        for (size_t i = 0; i < drivers.size(); ++i) {
                users[i].preferredId = static_cast<int>(i);
                users[i].iface.requestedId = [](mLockstepUser* ctx) {
                        auto* user = static_cast<SimpleLockstepUser*>(ctx);
                        return user->preferredId;
                };
                users[i].iface.sleep = nullptr;
                users[i].iface.wake = nullptr;
                GBASIOLockstepDriverCreate(&drivers[i], &users[i].iface);
                drivers[i].coordinator = &coordinator;
                GBASIOLockstepCoordinatorAttach(&coordinator, &drivers[i]);
                cores[i].core->setPeripheral(cores[i].core, mPERIPH_GBA_LINK_PORT, &drivers[i].d);
        }
}

void shutdownLockstep(GBASIOLockstepCoordinator& coordinator, std::array<GBASIOLockstepDriver, 2>& drivers) {
        for (auto& driver : drivers) {
                GBASIOLockstepCoordinatorDetach(&coordinator, &driver);
        }
        GBASIOLockstepCoordinatorDeinit(&coordinator);
}

} // namespace

int main(int argc, char** argv) {
#if defined(_WIN32)
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
        Options opts;
        if (!parseArgs(argc, argv, opts)) {
                usage(argv[0]);
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }

        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
                std::cerr << "Failed to initialize SDL: " << SDL_GetError() << "\n";
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }

        std::array<CoreWrapper, 2> cores{};
        if (!setupCore(opts, "gekkonet-0", cores[0]) || !setupCore(opts, "gekkonet-1", cores[1])) {
                teardownCore(cores[0]);
                teardownCore(cores[1]);
                SDL_Quit();
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }

        SDL_Window* window = SDL_CreateWindow("mGBA GekkoNet", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                static_cast<int>(cores[0].width * 2), static_cast<int>(cores[0].height * 2), SDL_WINDOW_RESIZABLE);
        SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        SDL_Texture* texture = renderer ? SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                static_cast<int>(cores[0].width), static_cast<int>(cores[0].height)) : nullptr;
        if (!window || !renderer || !texture) {
                std::cerr << "Failed to create SDL resources: " << SDL_GetError() << "\n";
                if (texture) {
                        SDL_DestroyTexture(texture);
                }
                if (renderer) {
                        SDL_DestroyRenderer(renderer);
                }
                if (window) {
                        SDL_DestroyWindow(window);
                }
                shutdownLockstep(coordinator, drivers);
                teardownCore(cores[0]);
                teardownCore(cores[1]);
                SDL_Quit();
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }

        GBASIOLockstepCoordinator coordinator;
        std::array<GBASIOLockstepDriver, 2> drivers{};
        std::array<SimpleLockstepUser, 2> users{};
        configureLockstep(coordinator, drivers, users, cores);

        GekkoSession* session = nullptr;
        if (!gekko_create(&session)) {
                std::cerr << "Failed to create GekkoNet session\n";
                shutdownLockstep(coordinator, drivers);
                teardownCore(cores[0]);
                teardownCore(cores[1]);
                SDL_DestroyTexture(texture);
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }

        GekkoConfig config{};
        config.num_players = 2;
        config.max_spectators = 0;
        config.input_prediction_window = static_cast<unsigned char>(opts.prediction);
        config.spectator_delay = 0;
        config.input_size = sizeof(uint16_t);
        config.state_size = kDefaultStateSize;
        config.limited_saving = false;
        config.post_sync_joining = false;
        config.desync_detection = true;

        GekkoNetAdapter* adapter = gekko_default_adapter(opts.localPort);
        if (!adapter) {
                std::cerr << "Failed to create GekkoNet adapter\n";
                gekko_destroy(session);
                shutdownLockstep(coordinator, drivers);
                teardownCore(cores[0]);
                teardownCore(cores[1]);
                SDL_DestroyTexture(texture);
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }
        gekko_net_adapter_set(session, adapter);

        int localHandle = gekko_add_actor(session, LocalPlayer, nullptr);

        sockaddr_in remoteAddr{};
        remoteAddr.sin_family = AF_INET;
        remoteAddr.sin_port = htons(opts.remotePort);
        if (inet_pton(AF_INET, opts.remoteHost.c_str(), &remoteAddr.sin_addr) != 1) {
                std::cerr << "Invalid remote host: " << opts.remoteHost << "\n";
                gekko_destroy(session);
                shutdownLockstep(coordinator, drivers);
                teardownCore(cores[0]);
                teardownCore(cores[1]);
                SDL_DestroyTexture(texture);
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }
        GekkoNetAddress addr{};
        addr.data = &remoteAddr;
        addr.size = sizeof(remoteAddr);
        int remoteHandle = gekko_add_actor(session, RemotePlayer, &addr);
        if (localHandle < 0 || remoteHandle < 0) {
                std::cerr << "Failed to register actors with GekkoNet\n";
                gekko_destroy(session);
                shutdownLockstep(coordinator, drivers);
                teardownCore(cores[0]);
                teardownCore(cores[1]);
                SDL_DestroyTexture(texture);
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
#if defined(_WIN32)
                WSACleanup();
#endif
                return EXIT_FAILURE;
        }

        gekko_start(session, &config);
        if (opts.localDelay) {
                gekko_set_local_delay(session, localHandle, static_cast<unsigned char>(opts.localDelay));
        }

        bool running = true;
        uint16_t localKeys = 0;
        int currentFrame = 0;
        std::unordered_map<int, FrameState> stateCache;
        std::vector<uint8_t> packBuffer;
        auto lastPresent = std::chrono::steady_clock::now();
        const double targetFrameMs = 1000.0 / 60.0;

        while (running) {
                SDL_Event event;
                while (SDL_PollEvent(&event)) {
                        if (event.type == SDL_QUIT) {
                                running = false;
                        } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
                                running = false;
                        } else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
                                localKeys = updateKeysFromEvent(event, localKeys);
                        }
                }

                gekko_add_local_input(session, localHandle, &localKeys);
                gekko_network_poll(session);

                int sessionEventsCount = 0;
                GekkoSessionEvent** sessionEvents = gekko_session_events(session, &sessionEventsCount);
                for (int i = 0; i < sessionEventsCount; ++i) {
                        auto* ev = sessionEvents[i];
                        if (!ev) {
                                continue;
                        }
                        switch (ev->type) {
                        case PlayerConnected:
                                std::cout << "Player " << ev->data.connected.handle << " connected\n";
                                break;
                        case PlayerDisconnected:
                                std::cout << "Player " << ev->data.disconnected.handle << " disconnected\n";
                                break;
                        case DesyncDetected:
                                std::cerr << "Desync detected at frame " << ev->data.desynced.frame << "\n";
                                break;
                        default:
                                break;
                        }
                }

                int eventCount = 0;
                GekkoGameEvent** events = gekko_update_session(session, &eventCount);
                for (int i = 0; i < eventCount; ++i) {
                        GekkoGameEvent* ev = events[i];
                        if (!ev) {
                                continue;
                        }
                        switch (ev->type) {
                        case SaveEvent: {
                                auto& save = ev->data.save;
                                uint32_t checksum = 0;
                                if (!packState(save.frame, cores[0].core, cores[1].core, packBuffer, checksum)) {
                                        std::cerr << "Failed to serialize state for frame " << save.frame << "\n";
                                        break;
                                }
                                if (packBuffer.size() > config.state_size) {
                                        std::cerr << "Serialized state exceeds configured size limit\n";
                                        break;
                                }
                                std::memcpy(save.state, packBuffer.data(), packBuffer.size());
                                *save.state_len = static_cast<unsigned int>(packBuffer.size());
                                *save.checksum = checksum;
                                stateCache[save.frame].data = packBuffer;
                                break;
                        }
                        case LoadEvent: {
                                auto& load = ev->data.load;
                                if (!loadPackedState(cores[0].core, cores[1].core, load.state, load.state_len)) {
                                        std::cerr << "Failed to load rollback state at frame " << load.frame << "\n";
                                }
                                currentFrame = load.frame;
                                break;
                        }
                        case AdvanceEvent: {
                                auto& adv = ev->data.adv;
                                const uint16_t* inputs = reinterpret_cast<const uint16_t*>(adv.inputs);
                                if (adv.input_len < sizeof(uint16_t) * 2) {
                                        break;
                                }
                                uint16_t mappedInputs[2];
                                mappedInputs[opts.localPlayer] = inputs[0];
                                mappedInputs[1 - opts.localPlayer] = inputs[1];
                                cores[0].core->setKeys(cores[0].core, mappedInputs[0]);
                                cores[1].core->setKeys(cores[1].core, mappedInputs[1]);
                                cores[0].core->runFrame(cores[0].core);
                                cores[1].core->runFrame(cores[1].core);
                                currentFrame = adv.frame;
                                auto now = std::chrono::steady_clock::now();
                                double elapsed = std::chrono::duration<double, std::milli>(now - lastPresent).count();
                                if (elapsed < targetFrameMs) {
                                        SDL_Delay(static_cast<Uint32>(targetFrameMs - elapsed));
                                }
                                updateTexture(texture, cores[0]);
                                SDL_RenderClear(renderer);
                                SDL_RenderCopy(renderer, texture, nullptr, nullptr);
                                SDL_RenderPresent(renderer);
                                lastPresent = std::chrono::steady_clock::now();
                                for (auto iter = stateCache.begin(); iter != stateCache.end();) {
                                        if (iter->first + 96 < adv.frame) {
                                                iter = stateCache.erase(iter);
                                        } else {
                                                ++iter;
                                        }
                                }
                                break;
                        }
                        default:
                                break;
                        }
                }
        }

        gekko_destroy(session);
        shutdownLockstep(coordinator, drivers);
        teardownCore(cores[0]);
        teardownCore(cores[1]);
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
#if defined(_WIN32)
        WSACleanup();
#endif
        return EXIT_SUCCESS;
}
