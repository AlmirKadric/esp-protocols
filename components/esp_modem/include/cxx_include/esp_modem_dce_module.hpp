/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <utility>
#include "generate/esp_modem_command_declare.inc"
#include "cxx_include/esp_modem_command_library.hpp"
#include "cxx_include/esp_modem_types.hpp"
#include "esp_modem_dce_config.h"

namespace esp_modem {

/**
 * @defgroup ESP_MODEM_MODULE
 * @brief Definition of modules representing specific modem devices
 */

/** @addtogroup ESP_MODEM_MODULE
* @{
*/

enum class command_result;
class DTE;
struct PdpContext;

/**
 * @brief This is a basic building block for custom modules as well as for the supported modules in the esp-modem component
 * It derives from the ModuleIf.
 */
class GenericModule: public ModuleIf {
public:
    /**
     * @brief We can construct a generic device with an existent DTE and it's configuration
     * The configuration could be either the dce-config struct or just a pdp context
     */
    explicit GenericModule(std::shared_ptr<DTE> dte, std::unique_ptr<PdpContext> pdp):
        dte(std::move(dte)), pdp(std::move(pdp)) {}
    explicit GenericModule(std::shared_ptr<DTE> dte, const esp_modem_dce_config *config);

    /**
     * @brief This is a mandatory method for ModuleIf class, which sets up the device
     * to be able to connect to the network. This typically consists of setting basic
     * communication parameters and setting the PDP (defining logical access point
     * to cellular network)
     */
    bool setup_data_mode() override
    {
        if (set_echo(false) != command_result::OK) {
            return false;
        }
        if (set_pdp_context(*pdp) != command_result::OK) {
            return false;
        }
        return true;
    }

    /**
     * @brief This is a mandatory method of ModuleIf class, which defines
     * basic commands for switching between DATA, COMMAND and CMUX modes
     */
    bool set_mode(modem_mode mode) override
    {
        if (mode == modem_mode::DATA_MODE) {
            if (set_data_mode() != command_result::OK) {
                return resume_data_mode() == command_result::OK;
            }
            return true;
        } else if (mode == modem_mode::COMMAND_MODE) {
            int retry = 0, status = 0;
            while (retry++ < 3) {
                // Mandatory 1s pause before escape
                Task::Delay(1000);

                // Create data and command callbacks
                // NOTE: even though the sequence is sent on the command
                // terminal, the responses can be received on the data terminal
                // where data mode is being terminated
                std::function<bool(uint8_t*, size_t)> dataCB = [&](uint8_t *data, size_t len){
                    std::string_view response((char*)data, len);
                    if (response.find("NO CARRIER", 0) != std::string::npos ||
                        response.find("OK", 0) != std::string::npos
                    ) {
                        status = 1;
                        return true;
                    }
                    if (response.find("ERROR", 0) != std::string::npos) {
                        status = -1;
                        return true;
                    }

                    return false;
                };
                std::function<command_result(uint8_t*, size_t)> commandCB = [&](uint8_t *data, size_t len){
                    if (dataCB(data, len)) {
                        return command_result::OK;
                    } else {
                        return command_result::TIMEOUT;
                    }
                };

                // Wait for response helper function
                std::function<void(int)> waitResp = [&](int delay){
                    for (int tick = 0; tick < (delay / 100) + 1; tick += 1) {
                        Task::Delay(100);
                        if (status != 0) {
                            break;
                        }
                    }
                };

                // Send the escape sequence to the command and data terminals
                // capturing data and command responses
                // NOTE: this includes the mandatory pauses after the sequence
                // via wait for response or response timeout
                std::string escapeSequence = "+++";

                status = 0;
                dte->set_read_cb(dataCB);
                dte->write((uint8_t*)escapeSequence.data(), escapeSequence.length());
                waitResp(5000);
                if (status != 0) {
                    return status == 1;
                }

                status = 0;
                dte->set_read_cb(dataCB);
                dte->command(escapeSequence, commandCB, 5000);
                if (status != 0) {
                    return status == 1;
                }

                // Send new line before attempting to check if terminals accept
                // AT commands
                std::string lineDelim = "\r\n";
                dte->set_read_cb(dataCB);
                dte->write((uint8_t*)lineDelim.data(), lineDelim.length());
                dte->command(lineDelim, commandCB, 1000);

                // Check if AT commands work directly on both data and command
                // terminals
                std::string syncCmd = "AT\r";

                status = 0;
                dte->set_read_cb(dataCB);
                dte->write((uint8_t*)syncCmd.data(), syncCmd.length());
                waitResp(1000);
                if (status == 1) {
                    status = 0;
                    dte->set_read_cb(dataCB);
                    dte->command(syncCmd, commandCB, 1000);
                    if (status == 1) {
                        return true;
                    }
                }
            }
            return false;
        } else if (mode == modem_mode::CMUX_MODE) {
            return set_cmux() == command_result::OK;
        }
        return true;
    }

    /**
     * @brief Additional method providing runtime configuration of PDP context
     */
    void configure_pdp_context(std::unique_ptr<PdpContext> new_pdp)
    {
        pdp = std::move(new_pdp);
    }

    /**
     * @brief Simplified version of operator name (without the ACT, which is included in the command library)
     */
    command_result get_operator_name(std::string &name)
    {
        int dummy_act;
        return get_operator_name(name, dummy_act);
    }

    /**
     * @brief Common DCE commands generated from the API AT list
     */
#define ESP_MODEM_DECLARE_DCE_COMMAND(name, return_type, num, ...) \
    virtual return_type name(__VA_ARGS__);

    DECLARE_ALL_COMMAND_APIS(virtual return_type name(...);)

#undef ESP_MODEM_DECLARE_DCE_COMMAND


protected:
    std::shared_ptr<DTE> dte;         /*!< Generic device needs the DTE as a channel talk to the module using AT commands */
    std::unique_ptr<PdpContext> pdp;  /*!< It also needs a PDP data, const information used for setting up cellular network */
};

// Definitions of other supported modules with some specific commands overwritten

/**
 * @brief Specific definition of the SIM7600 module
 */
class SIM7600: public GenericModule {
    using GenericModule::GenericModule;
public:
    command_result get_battery_status(int &voltage, int &bcs, int &bcl) override;
    command_result power_down() override;
    command_result set_gnss_power_mode(int mode) override;
    command_result set_network_bands(const std::string &mode, const int *bands, int size) override;
};

/**
 * @brief Specific definition of the SIM7070 module
 */
class SIM7070: public GenericModule {
    using GenericModule::GenericModule;
public:
    command_result power_down() override;
    command_result set_data_mode() override;

};

/**
 * @brief Specific definition of the SIM7000 module
 */
class SIM7000: public GenericModule {
    using GenericModule::GenericModule;
public:
    command_result power_down() override;
};

/**
 * @brief Specific definition of the SIM800 module
 */
class SIM800: public GenericModule {
    using GenericModule::GenericModule;
public:
    command_result power_down() override;
};

/**
 * @brief Specific definition of the BG96 module
 */
class BG96: public GenericModule {
    using GenericModule::GenericModule;
public:
    command_result set_pdp_context(PdpContext &pdp) override;
};

class SQNGM02S : public GenericModule {
    using GenericModule::GenericModule;

public:
    command_result connect(PdpContext &pdp);
    bool setup_data_mode() override;
};

/**
 * @}
 */

} // namespace esp_modem
