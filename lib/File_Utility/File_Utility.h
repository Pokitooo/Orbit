#ifndef FILE_UTILITY_H
#define FILE_UTILITY_H

#include <Arduino.h>
#include <Arduino_Extended.h>
#include <SdFat.h>

enum class FsMode : uint8_t {
    READ = 0,
    WRITE,
    APPEND
};

template<typename SdClass = SdExFat, typename FileClass = ExFile>
class FsUtil {
protected:
    SdCardFactory m_factory = {};
    SdCard *m_card          = {};
    uint32_t m_sector_count = {};
    uint8_t m_buf[512]      = {};

    SdClass m_sd            = {};
    FileClass m_file        = {};
    String m_filename       = {};

    struct ls_helper {
        SdClass &m_sd;
        const char *m_path;

        constexpr explicit ls_helper(SdClass &sd, const char *path) : m_sd{sd}, m_path{path} {}

        ls_helper &operator>>(Stream &stream) {
            FileClass root = m_sd.open(m_path);
            FileClass entry;
            while ((entry = root.openNextFile())) {
                char tmp[256];
                entry.getName(tmp, sizeof(tmp));
                if (entry.isFile()) {
                    stream.print("File: ");
                    stream.print(tmp);
                    stream.print("\tSize = ");
                    stream.print(entry.size());
                    stream.println(" bytes");
                } else if (entry.isDir()) {
                    stream.print("Dir:  ");
                    stream.println(tmp);
                }
                entry.close();
            }
            return *this;
        }
    };

public:
    FsUtil() {
        m_filename.reserve(64);
    }

    [[nodiscard]] ls_helper ls(const String &path) {
        return ls_helper(m_sd, path.c_str());
    }

    [[nodiscard]] ls_helper ls(const char *path = "/") {
        return ls_helper(m_sd, path);
    }

    constexpr SdClass &sd() {
        return m_sd;
    }

    constexpr FileClass &file() {
        return m_file;
    }

    template<FsMode Mode>
    void open_one() {
        m_file = open<Mode>(m_filename);
    }

    void close_one() {
        m_file.close();
    }

    void flush_one() {
        m_file.flush();
    }

    template<FsMode Mode>
    [[nodiscard]] FileClass open(const String &filename) {
        return open<Mode>(filename.c_str());
    }

    template<FsMode Mode>
    [[nodiscard]] FileClass open(const char *filename) {
        FileClass file;

        if constexpr (Mode == FsMode::READ) {
            file = m_sd.open(filename, FILE_READ);
        } else if constexpr (Mode == FsMode::WRITE) {
            file = m_sd.open(filename, FILE_WRITE);
        } else if constexpr (Mode == FsMode::APPEND) {
            file = m_sd.open(filename, FILE_WRITE);
            file.seekEnd();
        }

        return file;
    }

    void find_file_name(const char *prefix, const char *extension = "csv") {
        uint32_t file_idx = 1;
        do {
            m_filename = "";
            m_filename << prefix << file_idx++ << "." << extension;
        } while (m_sd.exists(m_filename.c_str()));
    }

    int8_t fmt_new_card(const SdSpiConfig &sd_config) {
        m_card = m_factory.newCard(sd_config);

        if (!m_card || m_card->errorCode()) {
            return 1;
        }

        m_sector_count = m_card->sectorCount();

        if (!m_sector_count) {
            return 2;
        }

        return 0;
    }

    int8_t fmt_erase_card() {
        static constexpr uint32_t ERASE_SIZE = 262144L;
        uint32_t first_block                 = 0;


        do {
            uint32_t last_block = first_block + ERASE_SIZE - 1;
            if (last_block >= m_sector_count) {
                last_block = m_sector_count - 1;
            }

            if (!m_card->erase(first_block, last_block)) {
                return 1;
            }

            first_block += ERASE_SIZE;
        } while (first_block < m_sector_count);

        if (!m_card->readSector(0, m_buf)) {
            return 2;
        }

        return 0;
    }

    int8_t fmt_format_card(Print &print) {
        if constexpr (std::is_same_v<SdExFat, SdClass>)
            return !ExFatFormatter().format(m_card, m_buf, &print);
        else
            return !FatFormatter().format(m_card, m_buf, &print);
    }
};

#endif  //FILE_UTILITY_H
