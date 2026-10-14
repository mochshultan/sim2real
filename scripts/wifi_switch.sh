#!/usr/bin/env bash
# Simpan IPv4 statis sekali, lalu pindah Wi-Fi menggunakan profil NetworkManager.
set -euo pipefail
export LC_ALL=C

ERC_PROFILE="erc_5G"
GUEST_PROFILE="ntnu_guest"

die() { printf 'Error: %s\n' "$*" >&2; exit 1; }

usage() {
    cat <<'EOF'
Pemakaian:
  ./scripts/wifi_switch.sh setup                     # Tanya IP/prefix, gateway, DNS
  ./scripts/wifi_switch.sh setup IP/PREFIX GATEWAY DNS
  ./scripts/wifi_switch.sh erc                       # Jadikan default saat boot dan sambung erc_5G
  ./scripts/wifi_switch.sh guest                     # Sambung ntnu_guest sementara
  ./scripts/wifi_switch.sh status                    # Lihat koneksi dan IP

DNS: satu atau beberapa IPv4, dipisahkan koma (contoh: 1.1.1.1,8.8.8.8).
Setup menyimpan konfigurasi permanen; tidak mengaktifkan koneksi.
IP harus sesuai subnet router erc_5G dan sudah dialokasikan untuk komputer ini.
Jalankan dengan sudo jika NetworkManager menolak izin.
EOF
}

uuid_for() {
    nmcli -g connection.uuid connection show id "$1" 2>/dev/null
}

show_status() {
    nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
    printf '\nAlamat, gateway, dan DNS aktif:\n'
    nmcli -f GENERAL.DEVICE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY,IP4.DNS device show
}

setup_erc() {
    command -v python3 >/dev/null || die 'python3 diperlukan untuk validasi alamat IP.'
    local address gateway dns uuid iface
    if [[ $# -eq 0 ]]; then
        read -r -p 'IP statis/prefix untuk erc_5G (contoh 192.168.1.50/24): ' address
        read -r -p 'Gateway router erc_5G (contoh 192.168.1.1): ' gateway
        read -r -p 'DNS IPv4, pisahkan koma jika lebih dari satu: ' dns
    elif [[ $# -eq 3 ]]; then
        address=$1 gateway=$2 dns=$3
    else
        die 'Gunakan setup tanpa argumen, atau setup IP/PREFIX GATEWAY DNS.'
    fi

    python3 - "$address" "$gateway" "$dns" <<'PY'
import ipaddress
import sys

try:
    address, gateway, dns = sys.argv[1:]
    if '/' not in address:
        raise ValueError('IP statis harus menyertakan prefix, misalnya /24.')
    interface = ipaddress.IPv4Interface(address)
    router = ipaddress.IPv4Address(gateway)
    if interface.network.prefixlen > 30:
        raise ValueError('Gunakan prefix 1 sampai 30 untuk jaringan Wi-Fi ini.')
    if interface.network.prefixlen == 0:
        raise ValueError('Prefix /0 tidak didukung.')
    for label, ip in [('IP statis', interface.ip), ('Gateway', router)]:
        if ip.is_unspecified or ip.is_multicast or ip.is_loopback or ip.is_link_local:
            raise ValueError(f'{label} tidak sesuai untuk jaringan Wi-Fi ini.')
        if ip not in interface.network or ip in (
            interface.network.network_address, interface.network.broadcast_address
        ):
            raise ValueError(f'{label} harus berupa alamat host dalam subnet {interface.network}.')
    if interface.ip == router:
        raise ValueError('IP statis tidak boleh sama dengan gateway.')
    for item in dns.split(','):
        server = ipaddress.IPv4Address(item.strip())
        if server.is_unspecified or server.is_multicast:
            raise ValueError('Alamat DNS tidak valid.')
except ValueError as error:
    sys.exit(f'Error: {error}')
PY

    dns=${dns//[[:space:]]/}
    if uuid=$(uuid_for "$ERC_PROFILE"); then
        [[ $(nmcli -g 802-11-wireless.ssid connection show uuid "$uuid") == "$ERC_PROFILE" ]] \
            || die 'Profil erc_5G tidak menunjuk SSID erc_5G.'
        nmcli connection modify uuid "$uuid" \
            ipv4.method manual ipv4.addresses "$address" \
            ipv4.gateway "$gateway" ipv4.dns "$dns" ipv4.ignore-auto-dns yes
    else
        iface=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" {print $1; exit}')
        [[ -n $iface ]] || die 'Interface Wi-Fi tidak ditemukan.'
        nmcli connection add type wifi ifname "$iface" con-name "$ERC_PROFILE" \
            ssid "$ERC_PROFILE" connection.autoconnect no \
            wifi-sec.key-mgmt wpa-psk \
            ipv4.method manual ipv4.addresses "$address" \
            ipv4.gateway "$gateway" ipv4.dns "$dns" ipv4.ignore-auto-dns yes
    fi
    printf '\nKonfigurasi tersimpan: IP=%s gateway=%s DNS=%s\n' "$address" "$gateway" "$dns"
    printf 'Untuk menyambung: %s erc\n' "$0"
}

prefer_erc() {
    local erc_uuid=$1 uuid type
    # Profil harus tersedia sebelum login, dengan prioritas tertinggi.
    nmcli connection modify uuid "$erc_uuid" \
        connection.permissions "" connection.autoconnect yes \
        connection.autoconnect-priority 999
    # Jaringan lain tetap dapat dipilih manual, tetapi tidak saat boot/reconnect.
    while IFS=: read -r uuid type; do
        if [[ $type == 802-11-wireless || $type == wifi ]]; then
            [[ $uuid == "$erc_uuid" ]] || \
                nmcli connection modify uuid "$uuid" connection.autoconnect no
        fi
    done < <(nmcli -t -f UUID,TYPE connection show)
}

switch_to() {
    local profile=$1 uuid method addresses
    uuid=$(uuid_for "$profile") || die "Profil $profile belum ada di NetworkManager."
    if [[ $profile == "$ERC_PROFILE" ]]; then
        method=$(nmcli -g ipv4.method connection show uuid "$uuid")
        addresses=$(nmcli -g ipv4.addresses connection show uuid "$uuid")
        [[ $method == manual && -n $addresses ]] \
            || die "IP statis belum diatur. Jalankan: $0 setup"
        prefer_erc "$uuid"
    fi
    printf 'Menghubungkan ke %s...\n' "$profile"
    if [[ -n ${SSH_CONNECTION:-} ]]; then
        printf 'Perpindahan Wi-Fi akan memutus sesi SSH ini; sambungkan kembali melalui IP jaringan tujuan.\n'
    fi
    nmcli --ask --wait 45 connection up uuid "$uuid"
    show_status
}

action=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
    help|-h|--help) usage; exit 0 ;;
    setup|erc|guest|status) ;;
    *) usage >&2; die "Perintah tidak dikenal: $action" ;;
esac
command -v nmcli >/dev/null || die 'nmcli tidak ditemukan; skrip memerlukan NetworkManager.'
[[ $action == setup || $# -eq 0 ]] || die "Perintah $action tidak menerima argumen tambahan."
case "$action" in
    setup) setup_erc "$@" ;;
    erc) switch_to "$ERC_PROFILE" ;;
    guest) switch_to "$GUEST_PROFILE" ;;
    status) show_status ;;
esac
