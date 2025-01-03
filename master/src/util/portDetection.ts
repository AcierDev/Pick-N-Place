import { SerialPort } from "serialport";
import chalk from "chalk";

export async function detectMicrocontrollerPort(): Promise<string | null> {
  try {
    const ports = await SerialPort.list();
    console.log(
      chalk.blue("Available ports:"),
      chalk.gray(JSON.stringify(ports, null, 2))
    );

    // Known USB-to-Serial converter IDs
    const knownVendorIds = [
      "1a86", // CH340
      "0403", // FTDI
      "10c4", // Silicon Labs CP210x
      "1781", // Multiple Arduino vendors
      "0483", // Arduino Uno
      "2341", // Arduino SA
      "2A03", // Arduino.org
    ];

    // First try to find a port by vendor ID
    const microcontrollerPort = ports.find((port) => {
      // Convert vendorId to lowercase if it exists
      const vendorId = port.vendorId?.toLowerCase();
      if (vendorId) {
        console.log(`Checking port ${port.path} with vendor ID: ${vendorId}`);
      }
      return vendorId && knownVendorIds.includes(vendorId);
    });

    if (microcontrollerPort) {
      console.log(
        chalk.green("✓ Found port by vendor ID:"),
        chalk.cyan(JSON.stringify(microcontrollerPort))
      );
      return microcontrollerPort.path;
    }

    // Fallback: look for common USB serial port patterns
    const serialPort = ports.find((port) => {
      const path = port.path.toLowerCase();
      console.log(`Checking path pattern for: ${path}`);
      return (
        path.includes("usbserial") ||
        path.includes("cu.wchusbserial") ||
        path.includes("ttyusb") ||
        path.includes("ttyacm") ||
        path.includes("cu.usbmodem") // Added for Arduino Uno
      );
    });

    if (serialPort) {
      console.log(
        chalk.green("✓ Found port by path pattern:"),
        chalk.cyan(JSON.stringify(serialPort))
      );
    } else {
      console.log(chalk.yellow("⚠ No suitable port found"));
    }

    return serialPort ? serialPort.path : null;
  } catch (error) {
    console.error(chalk.red("✗ Error detecting microcontroller port:"), error);
    return null;
  }
}
