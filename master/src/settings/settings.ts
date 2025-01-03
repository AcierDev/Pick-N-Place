import fs from "fs/promises";
import { SlaveSettings } from "../typings/types";

export class SettingsManager {
  private settingsPath: string;
  private settings: SlaveSettings;

  constructor(settingsPath: string) {
    this.settingsPath = settingsPath;
    this.settings = {
      sensorThreshold: 500,
      speed: 50,
      acceleration: 50,
      boxWidth: 20.0,
      boxLength: 20.0,
      boxX: 0,
      boxY: 0,
      pickupX: 0,
      pickupY: 0,
      rows: 1,
      columns: 1,
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
