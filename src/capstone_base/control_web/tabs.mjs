export const dashboardNames = ["wheels", "pan-tilt", "combined"];

// Only visible dashboards are loaded; combined explicitly loads both.
// Stop must be accepted before replacing it; failure keeps the current view.
export class DashboardTabs {
  constructor({send, display}) {
    this.send = send;
    this.display = display;
    this.selected = null;
    this.busy = false;
  }

  async select(name) {
    if (!dashboardNames.includes(name) || this.busy) return false;
    if (name === this.selected) return true;
    this.busy = true;
    try {
      const previous = this.selected;
      if (previous && !await this.send("switch", {from: previous, to: name})) return false;
      this.display(name, previous);
      this.selected = name;
      return true;
    } finally {
      this.busy = false;
    }
  }
}
