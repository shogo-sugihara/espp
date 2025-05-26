#pragma once

#include <mutex>
#include <vector>

#include <driver/spi_master.h>

#include "base_component.hpp"

#include "spi_format_helpers.hpp"

namespace espp {
/// @brief SPI driver
/// @details
/// This class is a wrapper around the ESP-IDF SPI driver.
///
/// \section Example
/// \snippet spi_example.cpp spi example
class Spi : public espp::BaseComponent {
public:
  /// Configuration for SPI
  struct Config {
    int isr_core_id = -1;        ///< The core to install the SPI interrupt on. If -1, then the SPI
                                 ///  interrupt is installed on the core that this constructor is
                                 ///  called on. If 0 or 1, then the SPI interrupt is installed on
                                 ///  the specified core.
    gpio_num_t sclk_io_num = GPIO_NUM_NC; ///< SCLK GPIO pin
    gpio_num_t mosi_io_num = GPIO_NUM_NC; ///< MOSI GPIO pin
    gpio_num_t miso_io_num = GPIO_NUM_NC; ///< MISO GPIO pin
    gpio_num_t cs_io_num = GPIO_NUM_NC;   ///< CS GPIO pin
    uint8_t mode = 0;                     ///< SPI mode (CPOL, CPHA)
    uint32_t clk_speed = 1000 * 1000;     ///< SPI clock speed in hertz
    int input_delay = 0;                  ///< Delay timing between SCLK and MISO to read data (ns)
    uint32_t flags = 0;                   ///< Bitwise OR of SPI_DEVICE_* flags
    int queue_size = 13;                  ///< Transaction queue size. This sets how many transactions can be 'in the air' (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
    bool auto_init = true;                ///< Automatically initialize I2C on construction
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Verbosity of logger
  };

  /// Construct SPI driver
  /// \param config Configuration for SPI
  explicit Spi(const Config &config)
      : BaseComponent("SPI", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
      if (ec) {
        logger_.error("auto init failed");
      }
    }
  }

  /// Destructor
  ~Spi() {
    std::error_code ec;
    deinit(ec);
    if (ec) {
      logger_.error("deinit failed");
    }
  }

  /// Initialize SPI driver
  void init(std::error_code &ec) {
    if (initialized_) {
      logger_.warn("already initialized");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }

    logger_.debug("Initializing SPI with config: {}", config_);

    esp_err_t ret;
    spi_bus_config_t bus_config;
    spi_device_interface_config_t device_config;

    memset(&bus_config, 0, sizeof(bus_config));
    memset(&device_config, 0, sizeof(device_config));

    std::unique_lock lock(mutex_);

    bus_config.sclk_io_num = config_.sclk_io_num;
    bus_config.mosi_io_num = config_.mosi_io_num;
    bus_config.miso_io_num = config_.miso_io_num;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;

    ret = spi_bus_initialize(host_, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
      logger_.error("bus config spi failed {}", esp_err_to_name(ret));
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    device_config.clock_speed_hz = config_.clk_speed;
    device_config.mode = config_.mode;
    device_config.spics_io_num = config_.cs_io_num;
    device_config.queue_size = config_.queue_size;
    device_config.flags = config_.flags;

    ret = spi_bus_add_device(host_, &device_config, &spi_);
    if (ret != ESP_OK) {
      logger_.error("device config spi failed {}", esp_err_to_name(ret));
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    logger_.info("SPI initialized {}", host_);

    initialized_ = true;
  }

  /// Deinitialize SPI driver
  void deinit(std::error_code &ec) {
    if (!initialized_) {
      logger_.warn("not initialized");
      // dont make this an error
      ec.clear();
      return;
    }
    esp_err_t ret;

    std::lock_guard<std::mutex> lock(mutex_);

    ret = spi_bus_remove_device(spi_);
    ret |= spi_bus_free(host_);
    if (ret != ESP_OK) {
      logger_.error("delete spi driver failed");
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    ec.clear();
    logger_.info("SPI deinitialized {}", host_);
    initialized_ = false;
  }

  /// Write to and read data from SPI device
  /// \param write_data Data to write
  /// \param read_data Data to read
  /// \param data_size Length of data to write/read
  /// \return True if successful
  bool write_read(const uint8_t *write_data, uint8_t *read_data, size_t data_size) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    logger_.debug("write and read for {} bytes", data_size);

    esp_err_t ret;
    struct spi_transaction_t transaction = {};
    transaction.length = data_size * 8;
    transaction.tx_buffer = write_data;
    transaction.rx_buffer = read_data;

    std::lock_guard<std::mutex> lock(mutex_);

    ret = spi_device_transmit(spi_, &transaction);
    if (ret != ESP_OK) {
      logger_.error("write read error: '{}'", esp_err_to_name(ret));
      return false;
    }

    return true;
  }

  /// Write data to and read data from SPI device
  /// \param write_data Data to write
  /// \param read_data Data to read
  /// \return True if successful
  bool write_read_vector(const std::vector<uint8_t> &write_data, std::vector<uint8_t> &read_data) {
    return write_read(write_data.data(), read_data.data(), write_data.size());
  }

protected:
  Config config_;
  bool initialized_ = false;
  std::mutex mutex_;
  spi_device_handle_t spi_;
  spi_host_device_t host_ = SPI2_HOST;
};
} // namespace espp

// for printing the SPI::Config using fmt
template <> struct fmt::formatter<espp::Spi::Config> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Spi::Config &c, FormatContext &ctx) const {
    // print the clock speed in khz
    auto clk_speed_khz = c.clk_speed / 1000;
    // if it's MHz, print it as such
    if (clk_speed_khz >= 1000) {
      clk_speed_khz = c.clk_speed / 1000000;
      return fmt::format_to(ctx.out(),
                            "SPI::Config{{SCLK: {}, MOSI: {}, MISO: {}, CS: {}, mode: {}, clock speed: {}MHz}}",
                            c.sclk_io_num, c.mosi_io_num, c.miso_io_num, c.cs_io_num, c.mode, clk_speed_khz);
    }
    return fmt::format_to(ctx.out(),
                          "SPI::Config{{SCLK: {}, MOSI: {}, MISO: {}, CS: {}, mode: {}, clock speed: {}kHz}}",
                          c.sclk_io_num, c.mosi_io_num, c.miso_io_num, c.cs_io_num, c.mode, clk_speed_khz);
  }
};
