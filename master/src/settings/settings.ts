import fs from "fs/promises";
import { SlaveSettings } from "../typings/types";

export class SettingsManager {
  private settingsPath: string;
  private settings: SlaveSettings;

  constructor(settingsPath: string) {
    this.settingsPath = settingsPath;
    this.settings = {
      sensorThreshold: 500,
    };
  }

  async loadSettings(): Promise<void> {
    try {
      const data = await fs.readFile(this.settingsPath, "utf-8");
      this.settings = JSON.parse(data);
    } catch (error) {
      console.error("Error loading settings:", error);
      await this.saveSettings();
    }
  }

  async saveSettings(): Promise<void> {
    try {
      await fs.writeFile(
        this.settingsPath,
        JSON.stringify(this.settings, null, 2)
      );
    } catch (error) {
      console.error("Error saving settings:", error);
    }
  }

  getSettings(): SlaveSettings {
    return this.settings;
  }

  updateSettings(newSettings: Partial<SlaveSettings>): void {
    this.settings = { ...this.settings, ...newSettings };
    this.saveSettings();
  }
}
