#!/usr/bin/env bash
# Real-time tuning for the 1 kHz FCI link. Run as root (run_server.sh calls it with sudo). Resets on reboot.
#   - robot NIC: energy-efficient ethernet off, flow control (PAUSE frames) off, PCIe ASPM off
#   - CPU idle states with exit latency > 1 us disabled     (deep C-states took ~3 ms to wake on the laptop)
#   - Realtek rtw89 Wi-Fi power saving off, if present      (its PCIe reads stalled a CPU for ~3 ms)
#   - ksoftirqd of the robot NIC's IRQ CPU at SCHED_FIFO 90, unbound kernel work kept off that CPU
# Usage: setup_rt.sh [IFACE]   (default: the interface that routes to $ROBOT_IP, default 172.16.0.2)
# Prints the NIC IRQ CPU on the last line as "CONTROL_CPU=<n>".
set -u
ROBOT_IP=${ROBOT_IP:-172.16.0.2}
IFACE=${1:-$(ip route get "$ROBOT_IP" 2>/dev/null | grep -oP 'dev \K\S+')}
[ "$(id -u)" -eq 0 ] || { echo "[setup_rt] must run as root"; exit 1; }
[ -n "$IFACE" ] || { echo "[setup_rt] no interface routes to $ROBOT_IP (is the robot cable plugged in?)"; exit 1; }
echo "[setup_rt] robot NIC: $IFACE ($(ethtool -i "$IFACE" 2>/dev/null | sed -n 's/^driver: //p'), route to $ROBOT_IP)"

ethtool --set-eee "$IFACE" eee off 2>/dev/null
echo "[setup_rt] $IFACE EEE: $(ethtool --show-eee "$IFACE" 2>/dev/null | sed -n 's/.*EEE status: //p')"
ethtool -A "$IFACE" autoneg off rx off tx off 2>/dev/null
echo "[setup_rt] $IFACE flow control: $(ethtool -a "$IFACE" 2>/dev/null | awk '/^(RX|TX):/ {printf "%s%s ", $1, $2}')"

# PCIe ASPM on the robot NIC: a link waking from L1 delays packets. Clear the ASPM control bits (LnkCtl[1:0]).
pdev=$(basename "$(readlink -f /sys/class/net/"$IFACE"/device)")
if lnk=$(setpci -s "$pdev" CAP_EXP+0x10.w 2>/dev/null); then
  if (( 0x$lnk & 3 )); then setpci -s "$pdev" CAP_EXP+0x10.w="$(printf %04x $(( 0x$lnk & ~3 )))"; fi
  echo "[setup_rt] $IFACE PCIe ASPM control: 0x$lnk -> 0x$(setpci -s "$pdev" CAP_EXP+0x10.w) (bits 1:0 = 0 means off)"
fi

# Keep only POLL and C1 (halt, 1 us). rtla timerlat measured a 2857 us wake-up from cpuidle with C3 already off:
# on this Ryzen laptop ACPI C2 (advertised 18 us) leads into deep core/package C-states that take milliseconds.
n=0
for st in /sys/devices/system/cpu/cpu*/cpuidle/state*; do
  [ "$(cat "$st/latency")" -gt 1 ] && echo 1 > "$st/disable" && n=$((n + 1))
done
echo "[setup_rt] disabled $n idle states (exit latency > 1 us): $(cat /sys/devices/system/cpu/cpu0/cpuidle/state*/name | paste -sd' ') -> $(for s in /sys/devices/system/cpu/cpu0/cpuidle/state*; do [ "$(cat $s/disable)" = 1 ] && cat $s/name; done | paste -sd' ') off"

# ---- Wi-Fi (Realtek RTL8852AE, rtw89): rtla timerlat caught rtw89_track_work -> rtw89_pci_ops_read32 stalling a
# CPU for 2949 us (a PCIe register read to a card in a low-power link state). Wi-Fi carries the SSH tunnel to the
# policy server, so keep it but turn its power saving off. Persisted in /etc/modprobe.d; reload only if needed.
WIFI_CONF=/etc/modprobe.d/rtw89-realtime.conf
if [ -d /sys/module/rtw89_pci ]; then
  if [ ! -f "$WIFI_CONF" ]; then
    printf '%s\n' "# written by policy_bridge/setup_rt.sh: avoid ms-long PCIe stalls during the 1 kHz robot loop" \
      "options rtw89_pci disable_aspm_l1=Y disable_aspm_l1ss=Y disable_clkreq=Y" \
      "options rtw89_core disable_ps_mode=Y" > "$WIFI_CONF"
  fi
  if [ "$(cat /sys/module/rtw89_pci/parameters/disable_aspm_l1)" != "Y" ] \
     || [ "$(cat /sys/module/rtw89_core/parameters/disable_ps_mode)" != "Y" ]; then
    echo "[setup_rt] reloading Wi-Fi driver with power saving off (Wi-Fi drops for a few seconds) ..."
    mods=$(lsmod | awk '/^rtw89_/ {print $1}')
    modprobe -r $(echo "$mods" | grep -E '8852ae$') rtw89_8852a rtw89_pci rtw89_core 2>&1 | sed 's/^/[setup_rt]   /'
    for m in $mods; do modprobe "$m"; done
    for _ in $(seq 1 60); do                       # wait up to 30 s for NetworkManager to reconnect
      nmcli -t -f TYPE,STATE device 2>/dev/null | grep -q '^wifi:connected$' && break
      sleep 0.5
    done
  fi
  echo "[setup_rt] Wi-Fi rtw89: aspm_l1 off=$(cat /sys/module/rtw89_pci/parameters/disable_aspm_l1)" \
       "l1ss off=$(cat /sys/module/rtw89_pci/parameters/disable_aspm_l1ss)" \
       "clkreq off=$(cat /sys/module/rtw89_pci/parameters/disable_clkreq)" \
       "ps_mode off=$(cat /sys/module/rtw89_core/parameters/disable_ps_mode)," \
       "link: $(nmcli -t -f TYPE,STATE,CONNECTION device 2>/dev/null | grep '^wifi:' | head -1)"
  for w in $(iw dev 2>/dev/null | awk '/Interface/ {print $2}'); do iw dev "$w" set power_save off 2>/dev/null; done
fi

# All IRQs of the robot NIC (one for r8169 "eno1"; several for multi-queue NICs such as Intel "eno1-TxRx-0..n")
# are moved to ONE CPU: the control CPU. The 1 kHz control process is pinned there too, so a robot packet wakes it
# locally. Default: the CPU the first NIC IRQ already uses; override with CONTROL_CPU=<n>.
irqs=$(awk -v i="$IFACE" '$NF==i || index($NF, i"-")==1 {sub(":","",$1); print $1}' /proc/interrupts)
[ -n "$irqs" ] || { echo "[setup_rt] no IRQ found for $IFACE"; exit 1; }
first=$(echo "$irqs" | head -1)
cpu=${CONTROL_CPU:-$(cut -d, -f1 /proc/irq/"$first"/effective_affinity_list | cut -d- -f1)}
for irq in $irqs; do
  echo "$cpu" > /proc/irq/"$irq"/smp_affinity_list 2>/dev/null
  for t in $(pgrep "^irq/$irq-"); do chrt -f -p 90 "$t"; done   # threaded IRQ handler (most drivers on PREEMPT_RT)
done
pid=$(pgrep -x "ksoftirqd/$cpu")
chrt -f -p 90 "$pid"
echo "[setup_rt] $IFACE IRQs $(echo $irqs) -> CPU $cpu ($(for i in $irqs; do cat /proc/irq/$i/effective_affinity_list; done | sort -u | paste -sd,));" \
     "ksoftirqd/$cpu: $(ps -o cls=,rtprio= -p "$pid")"

# Keep unbound kernel work (kworkers: Wi-Fi tracking, disk, ...) off the control CPU.
ncpu=$(nproc --all)
mask=$(printf '%x' $(( ((1 << ncpu) - 1) & ~(1 << cpu) )))
echo "$mask" > /sys/devices/virtual/workqueue/cpumask 2>/dev/null
echo "[setup_rt] unbound workqueue cpumask: $(cat /sys/devices/virtual/workqueue/cpumask) (CPU $cpu excluded)"
echo "CONTROL_CPU=$cpu"
