import { exec } from "child_process";
import { promisify } from "util";
import path from "path";

const execAsync = promisify(exec);

export class PlatformIOManager {
  private slavePath: string;

  constructor(slavePath: string) {
    this.slavePath = slavePath;
  }

  async uploadCode(): Promise<boolean> {
    try {
      console.log("Building and uploading slave code...");

      // Run PlatformIO upload command
      const { stdout, stderr } = await execAsync("pio run -t upload", {
        cwd: this.slavePath,
      });

      if (stderr && !stderr.includes("Success")) {
        console.error("Error uploading code:", stderr);
        return false;
      }

      console.log("Code uploaded successfully");
      return true;
    } catch (error) {
      console.error("Failed to upload code:", error);
      return false;
    }
  }

  async verifyPlatformIO(): Promise<boolean> {
    try {
      await execAsync("pio --version");
      return true;
    } catch (error) {
      console.error("PlatformIO CLI not found. Please install it first.");
      return false;
    }
  }
}
