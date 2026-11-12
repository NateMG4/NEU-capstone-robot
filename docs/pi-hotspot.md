# Pi Wi-Fi hotspot

The recorded bench configuration enables autoconnect for **Capstone-Pi**.
Wi-Fi association/SSH and startup after reboot still require confirmation on
the actual Pi. Join that Wi-Fi network from the laptop, then connect directly:

```bash
ssh user@10.42.0.1
```

The Wi-Fi password is in Obsidian at **Capstone → Code Reference → Pi - WiFi**.
It is separate from the Pi login password. On the Pi, display it with:

```bash
sudo nmcli --show-secrets -g 802-11-wireless-security.psk connection show capstone-hotspot
```

The laptop receives a `10.42.0.x` address automatically. No router or Ethernet
cable is needed for local robot access. Without an Internet uplink, the laptop
may report “No Internet”; SSH and local pages still work. NetworkManager shared
mode can also share the Pi's wired uplink when one is available.

## Run project commands

This is the **bench** network (`10.42.0.0/24`). The vision team's proposed
deployment network uses a master Pi access point (`capstone-robots`) and
`192.168.50.0/24`. These are separate configurations. On the recorded Pi 3,
one Wi-Fi radio cannot serve this hotspot and simultaneously act as the robot
client on that network. Agree on the lab topology before changing profiles;
see [camera integration status](camera-integration.md#verified-upstream-state-and-deployment-gaps).

After connecting by SSH:

```bash
cd ~/capstone-base
.venv/bin/capstone-base --vx 0.1
.venv/bin/capstone-teleop
```

Both commands use the fake backend. See [teleop](teleop.md), [servo](servo.md),
and [the dashboard guide](odometry-dashboard.md) for hardware commands.
For a dashboard listening on `0.0.0.0:8081`, open `http://10.42.0.1:8081`.

For uploads, use `user@10.42.0.1:~/capstone-base/` as the rsync destination.
The current WSL SSH aliases `capstone-pi.local` and `capstone-pi` pin the old
wired address, `129.10.156.234`. Use the numerical Wi-Fi address on the hotspot.

## Start and stop

Run these on the Pi. Normal startup needs only Pi power; autoconnect is enabled.

```bash
# Start manually
sudo nmcli radio wifi on
sudo nmcli connection up capstone-hotspot

# Stop for this session (disconnects Wi-Fi SSH clients)
sudo nmcli connection down capstone-hotspot

# Disable automatic startup, then stop
sudo nmcli connection modify capstone-hotspot connection.autoconnect no
sudo nmcli connection down capstone-hotspot

# Restore automatic startup and start now
sudo nmcli connection modify capstone-hotspot connection.autoconnect yes
sudo nmcli radio wifi on
sudo nmcli connection up capstone-hotspot
```

After stopping Wi-Fi, reconnect over Ethernet to start it again. The wired
NetworkManager profile remains available alongside the hotspot. The Pi's saved
phone Wi-Fi profile also remains, with lower autoconnect priority; its one Wi-Fi
radio serves the hotspot while this profile is active.

## Configuration and troubleshooting

Configured 2026-10-29 on Raspberry Pi 3 Model B Rev 1.2, Debian 13 (trixie),
NetworkManager 1.52.1:

| Setting | Value |
|---|---|
| Connection profile | `capstone-hotspot` |
| Interface / SSID | `wlan0` / `Capstone-Pi` |
| Band / channel | 2.4 GHz / 6 |
| Security | WPA2 Personal / CCMP |
| PMF | Disabled (`1`), required on this Pi for successful AP activation |
| Power saving | Disabled (`2`) |
| Pi address | `10.42.0.1/24` |
| IPv4 method | `shared` (NetworkManager-managed DHCP/DNS) |
| IPv6 | Disabled on this profile |
| Autoconnect / priority | Yes / `100` |

Verified access point activation, DHCP/DNS listeners, SSH listening on all IPv4
interfaces, saved autoconnect settings, and a stop/start cycle. Windows detected
the network at 91% signal. Laptop association/SSH over Wi-Fi and a physical
reboot were not performed during setup.

Check status on the Pi:

```bash
nmcli -f DEVICE,TYPE,STATE,CONNECTION device
ip -4 address show wlan0
sudo /usr/sbin/iw dev wlan0 info
sudo journalctl -u NetworkManager --since '-5 minutes' --no-pager
```

Expected: `wlan0` connected to `capstone-hotspot`, `10.42.0.1/24`, and type `AP`.
The initial activation failed with a supplicant timeout and kernel “key setting
validation failed”; disabling PMF resolved it while retaining WPA2 encryption.
If this recurs after recreating the profile:

```bash
sudo nmcli connection modify capstone-hotspot 802-11-wireless-security.pmf 1
sudo nmcli connection down capstone-hotspot
sudo nmcli connection up capstone-hotspot
```

This matches the failure documented in the
[Raspberry Pi driver issue](https://github.com/raspberrypi/linux/issues/7247).
See also the [NetworkManager command reference](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nmcli.html)
and [shared IPv4 settings](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html).

To remove the hotspot permanently, first connect over Ethernet, then run
`sudo nmcli connection delete capstone-hotspot`.
