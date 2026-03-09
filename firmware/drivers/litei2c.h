//
// Created by ts on 6/17/25.
//

#ifndef LITEI2C_H
#define LITEI2C_H
#include <stdint.h>

typedef struct {
    uint32_t master_active_addr;
    uint32_t master_addr_addr;
    uint32_t master_settings_addr;
    uint32_t master_status_addr;
    uint32_t master_rxtx_addr;
} litei2c_regs;

#define MASTER_STATUS_TX_READY_OFFSET 0x0
#define MASTER_STATUS_RX_READY_OFFSET 0x1
#define MASTER_STATUS_NACK_OFFSET     0x8

/**
 * @brief Performs an I2C transfer operation on the specified I2C bus.
 *
 * This function initiates an I2C transfer using the provided register set. It
 * supports both write (transmit) and read (receive) operations, depending on the
 * specified transmit length (txlen) and receive length (rxlen).
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param buf Pointer to a buffer that holds data for transmission or will contain
 *            data received during the operation.
 *            Received data is at the BOTTOM of the buffer, i.e., if txlen=2 and rxlen=2
 *            Received data will be in buf[0:15].
 *            But data is written and read from the top of the valid buffer.
 *            If txlen=2, then the first byte to be sent is buf[8:15]
 *            If txlen=3, then the first byte to be sent is buf[16:23]
 *            etc…
 *            The Same happens with RX.
 * @param txlen Number of bytes to be transmitted from the buffer.
 * @param rxlen Number of bytes to be received and stored in the buffer.
 * @return Status of the I2C transfer operation. A return of 0 indicates
 *         success, while a non-zero value indicates an error during the transfer.
 */
int8_t litei2c_transfer(const litei2c_regs *regs, uint8_t I2C_addr, uint32_t *buf, uint8_t txlen, uint8_t rxlen);

/**
 * @brief Performs an I2C transfer operation on the specified I2C bus and reorders data.
 *
 * This function is a wrapper of litei2c_transfer() that reorders data so that when
 * transmitting, LSB is always the first byte to be sent, and when receiving, LSB is always
 * the first byte that was received
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param buf Pointer to a buffer that holds data for transmission or will contain
 *            data received during the operation.
 *            Received data is at the BOTTOM of the buffer i.e. if txlen=2 and rxlen=2
 *            Received data will be in buf[0:15].
 * @param txlen Number of bytes to be transmitted from the buffer.
 * @param rxlen Number of bytes to be received and stored in the buffer.
 * @return Status of the I2C transfer operation. A return of 0 indicates
 *         success, while a non-zero value indicates an error during the transfer.
 */
int8_t litei2c_transfer_reordered(const litei2c_regs *regs, uint8_t I2C_addr, uint32_t *buf, uint8_t txlen, uint8_t rxlen);

/**
 * @brief Recovers an I2C bus using the provided register set.
 *
 * This function attempts to recover the I2C bus that may have been left in a
 * stalled or busy state. It uses the provided register set to perform the necessary
 * actions to restore the bus to an operational state.
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller registers.
 * @return Status indicating the result of the recovery process. A return value
 *         of 0 indicates a successful recovery, while non-zero values
 *         signify an error or failure to recover the bus.
 */
int8_t litei2c_recover_bus(const litei2c_regs* regs);

/**
 * @brief Reads an 8-bit register from an I2C device with an 8-bit register address.
 *
 * This function performs a single transfer to read the value of a specified 8-bit
 * register from a device on the I2C bus. It uses the provided I2C controller register
 * set to perform the operation and stores the value of the register in the pointed location.
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr 8-bit address of the register to be read from the I2C device.
 * @param reg_val Pointer to an 8-bit variable where the read register value will be stored.
 * @return Returns 0 if the register read operation is successful. A negative value
 *         is returned in case of an error (e.g., NACK received).
 */
int8_t litei2c_a8d8_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint8_t reg_addr, uint8_t *reg_val);

/**
 * @brief Writes a value to an 8-bit register of an I2C device with an 8-bit register address.
 *
 * This function performs a register write operation to the I2C device
 * by sending the register address and the value to be written. The
 * operation uses the I2C driver to transfer the data to the target device.
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller registers.
 * @param I2C_addr 7-bit I2C address of the target I2C device.
 * @param reg_addr 8-bit address of the register to be written to in the I2C device.
 * @param reg_val 8-bit value to be written to the specified register.
 * @return Status of the register write operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a8d8_write_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint8_t reg_addr, const uint8_t reg_val);

/**
 * @brief Reads an 8-bit value from a 16-bit register of an I2C device.
 *
 * This function performs a read operation from a specific 16-bit register
 * of an I2C device with a 7-bit I2C address. The read value is stored in
 * the location pointed to by `reg_val`.
 *
 * Register address is transmitted MSB-first
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr 16-bit address of the register to be read.
 * @param reg_val Pointer to a variable where the 8-bit value read from the
 *                register will be stored.
 * @return Status of the register write operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a16d8_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint16_t reg_addr, uint8_t *reg_val);

/**
 * @brief Writes an 8-bit value to a 16-bit register address on an I2C device.
 *
 * This function performs an I2C write operation where the target register address
 * is 16 bits wide, and the value to be written is 8 bits wide.
 *
 * Register address is transmitted MSB-first
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller's registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr 16-bit register address to which the value will be written.
 * @param reg_val 8-bit value to be written to the specified register.
 * @return Status of the register write operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a16d8_write_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint16_t reg_addr, const uint8_t reg_val);

/**
 * @brief Reads a 16-bit register value from a device over I2C using an 8-bit register address.
 *
 * This function reads a 16-bit value stored at the specified 8-bit register address (reg_addr)
 * on a peripheral device addressed by its 7-bit I2C address (I2C_addr). The register value
 * is retrieved through the I2C interface controlled by the provided I2C register structure.
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller's registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr 8-bit address of the register to read on the target device.
 * @param reg_val Pointer to an uint16_t variable where the read register value will be stored.
 * @return Status of the register write operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a8d16_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint8_t reg_addr, uint16_t *reg_val);

/**
 * @brief Writes a 16-bit value to an 8-bit register on a target device via I2C.
 *
 * This function writes a 16-bit value to a specific 8-bit register on a device
 * identified by its I2C address. Communication is performed using the provided
 * memory-mapped register set of the I2C controller.
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller's registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr The 8-bit address of the register within the target device
 *                 where the 16-bit value will be written.
 * @param reg_val The 16-bit value to be written into the specified device register.
 * @return Status of the register write operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a8d16_write_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint8_t reg_addr, const uint16_t reg_val);

/**
 * @brief Reads a 16-bit register value from a device over I2C using a 16-bit register address.
 *
 * This function reads a 16-bit value stored at the specified 16-bit register address (reg_addr)
 * on a peripheral device addressed by its 7-bit I2C address (I2C_addr). The register value
 * is retrieved through the I2C interface controlled by the provided I2C register structure.
 *
 * Register address is transmitted MSB-first
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller's registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr 16-bit address of the register to read on the target device.
 * @param reg_val Pointer to an uint16_t variable where the read register value will be stored.
 * @return Status of the register read operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a16d16_read_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint16_t reg_addr, uint16_t *reg_val);

/**
 * @brief Writes a 16-bit value to a 16-bit register on a target device via I2C.
 *
 * This function writes a 16-bit value to a specific 16-bit register on a device
 * identified by its I2C address. Communication is performed using the provided
 * memory-mapped register set of the I2C controller.
 *
 * Register address is transmitted MSB-first
 *
 * @param regs Pointer to a litei2c_regs structure containing the memory-mapped
 *             addresses of the I2C controller's registers.
 * @param I2C_addr 7-bit I2C address of the target device.
 * @param reg_addr The 16-bit address of the register within the target device
 *                 where the 16-bit value will be written.
 * @param reg_val The 16-bit value to be written into the specified device register.
 * @return Status of the register write operation. A return value of 0
 *         indicates success, while a non-zero value indicates an error during
 *         the transaction (e.g., if the device does not ACK the operation).
 */
int8_t litei2c_a16d16_write_register(const litei2c_regs *regs, const uint8_t I2C_addr, const uint16_t reg_addr, const uint16_t reg_val);

#endif //LITEI2C_H
