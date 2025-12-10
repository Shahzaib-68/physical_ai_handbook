// WebSerial/WebUSB integration for hardware connection demos
// Note: These APIs have limited browser support and require secure contexts (HTTPS)

import { getAppConfig } from '../utils/config';
import { DemoExecutionError } from '../utils/validation';

// Interface for device connection options
interface ConnectionOptions {
  baudRate?: number;
  dataBits?: number;
  stopBits?: number;
  parity?: 'none' | 'even' | 'odd';
  bufferSize?: number;
}

// Interface for WebUSB device info
interface USBDeviceInfo {
  vendorId: number;
  productId: number;
  productName: string;
  manufacturerName: string;
  serialNumber: string;
}

// Interface for WebSerial device info
interface SerialDeviceInfo {
  usbVendorId?: number;
  usbProductId?: number;
  baudRate: number;
}

// WebSerial API integration
export class WebSerialManager {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader | null = null;
  
  // Check if Web Serial API is supported
  static isSupported(): boolean {
    return 'serial' in navigator;
  }
  
  // Request a serial port
  async requestPort(options?: SerialPortRequestOptions): Promise<void> {
    if (!WebSerialManager.isSupported()) {
      throw new DemoExecutionError('Web Serial API not supported in this browser');
    }
    
    try {
      this.port = await navigator.serial.requestPort(options);
      await this.port.open({ baudRate: 9600 }); // Default baud rate
    } catch (error) {
      throw new DemoExecutionError(`Failed to request serial port: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Open port with specific options
  async openPort(options: ConnectionOptions = {}): Promise<void> {
    if (!this.port) {
      throw new DemoExecutionError('No port selected. Call requestPort first.');
    }
    
    const portOptions: SerialOptions = {
      baudRate: options.baudRate || 9600,
      dataBits: options.dataBits || 8,
      stopBits: options.stopBits || 1,
      parity: options.parity || 'none',
      bufferSize: options.bufferSize || 255
    };
    
    try {
      await this.port.open(portOptions);
    } catch (error) {
      throw new DemoExecutionError(`Failed to open serial port: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Read data from the serial port
  async read(): Promise<string> {
    if (!this.port) {
      throw new DemoExecutionError('No port opened');
    }
    
    // Create a reader if it doesn't exist
    if (!this.reader) {
      const readableStream = this.port.readable;
      if (readableStream) {
        this.reader = readableStream.getReader();
      } else {
        throw new DemoExecutionError('Port readable stream is not available');
      }
    }
    
    try {
      const { value, done } = await this.reader.read();
      if (done) {
        this.reader.releaseLock();
        return '';
      }
      
      // Convert bytes to string
      const decoder = new TextDecoder();
      return decoder.decode(value);
    } catch (error) {
      throw new DemoExecutionError(`Failed to read from serial port: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Write data to the serial port
  async write(data: string | Uint8Array): Promise<void> {
    if (!this.port) {
      throw new DemoExecutionError('No port opened');
    }
    
    const writer = this.port.writable?.getWriter();
    if (!writer) {
      throw new DemoExecutionError('Port writable stream is not available');
    }
    
    try {
      // Convert string to Uint8Array if needed
      const buffer = typeof data === 'string' ? new TextEncoder().encode(data) : data;
      
      await writer.write(buffer);
      await writer.close();
    } catch (error) {
      throw new DemoExecutionError(`Failed to write to serial port: ${error instanceof Error ? error.message : 'Unknown error'}`);
    } finally {
      writer.releaseLock();
    }
  }
  
  // Close the serial port
  async close(): Promise<void> {
    if (this.reader) {
      await this.reader.cancel();
      this.reader = null;
    }
    
    if (this.port) {
      await this.port.close();
      this.port = null;
    }
  }
  
  // Get port info
  getInfo(): SerialDeviceInfo | null {
    if (!this.port) return null;
    
    return {
      usbVendorId: this.port.getInfo().usbVendorId,
      usbProductId: this.port.getInfo().usbProductId,
      baudRate: this.port.getInfo().baudRate || 9600
    };
  }
}

// WebUSB API integration
export class WebUSBManager {
  // Check if Web USB API is supported
  static isSupported(): boolean {
    return 'usb' in navigator;
  }
  
  // Request a USB device
  async requestDevice(filters: USBDeviceFilter[] = []): Promise<USBDeviceInfo> {
    if (!WebUSBManager.isSupported()) {
      throw new DemoExecutionError('Web USB API not supported in this browser');
    }
    
    try {
      const device = await (navigator as any).usb.requestDevice({ filters });
      
      return {
        vendorId: device.vendorId,
        productId: device.productId,
        productName: device.productName,
        manufacturerName: device.manufacturerName,
        serialNumber: device.serialNumber
      };
    } catch (error) {
      throw new DemoExecutionError(`Failed to request USB device: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Get a list of connected USB devices
  async getDevices(): Promise<USBDeviceInfo[]> {
    if (!WebUSBManager.isSupported()) {
      throw new DemoExecutionError('Web USB API not supported in this browser');
    }
    
    try {
      const devices = await (navigator as any).usb.getDevices();
      
      return devices.map(device => ({
        vendorId: device.vendorId,
        productId: device.productId,
        productName: device.productName,
        manufacturerName: device.manufacturerName,
        serialNumber: device.serialNumber
      }));
    } catch (error) {
      throw new DemoExecutionError(`Failed to list USB devices: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Open a USB device
  async openDevice(vendorId: number, productId: number): Promise<void> {
    if (!WebUSBManager.isSupported()) {
      throw new DemoExecutionError('Web USB API not supported in this browser');
    }
    
    try {
      const devices = await (navigator as any).usb.getDevices();
      const device = devices.find((d: any) => 
        d.vendorId === vendorId && d.productId === productId
      );
      
      if (!device) {
        throw new DemoExecutionError(`Device with vendorId ${vendorId} and productId ${productId} not found`);
      }
      
      await device.open();
      await device.selectConfiguration(1);
      await device.claimInterface(0);
    } catch (error) {
      throw new DemoExecutionError(`Failed to open USB device: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Transfer data to USB device
  async transferOut(interfaceId: number, data: Uint8Array): Promise<void> {
    if (!WebUSBManager.isSupported()) {
      throw new DemoExecutionError('Web USB API not supported in this browser');
    }
    
    // Note: This would require an opened device, which we don't maintain in this class
    // for simplicity of the demo
    console.log(`Would transfer data to interface ${interfaceId}:`, data);
  }
  
  // Transfer data from USB device
  async transferIn(interfaceId: number, length: number): Promise<Uint8Array> {
    if (!WebUSBManager.isSupported()) {
      throw new DemoExecutionError('Web USB API not supported in this browser');
    }
    
    // Note: This would require an opened device, which we don't maintain in this class
    // for simplicity of the demo
    console.log(`Would read ${length} bytes from interface ${interfaceId}`);
    return new Uint8Array(0);
  }
}

// Validate WebSerial/WebUSB code (basic validation)
export const validateHardwareConnection = (code: string): string[] => {
  const errors: string[] = [];

  // These APIs are experimental and have security considerations
  if (code.includes('navigator.serial') || code.includes('navigator.usb')) {
    errors.push('Hardware connection APIs require secure context (HTTPS) and user permission');
  }

  return errors;
};

// Get the status of hardware APIs in the current environment
export const getHardwareAPIStatus = () => {
  return {
    webSerialSupported: WebSerialManager.isSupported(),
    webUsbSupported: WebUSBManager.isSupported(),
    isSecureContext: window.isSecureContext
  };
};