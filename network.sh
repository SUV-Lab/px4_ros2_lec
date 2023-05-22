#!/bin/bash
interface="any"
tcpdump -i $interface -w rtps_packets.pcap 'udp and (udp[8:4] = 0x52545053 or udp[12:4] = 0x52545053)' &> /dev/null &
echo "Packet collection started. Press Ctrl+C to stop."
trap "cleanup" INT
function cleanup() {
  kill $! &> /dev/null
  echo "Packet collection stopped."
  packet_count=$(tcpdump -r rtps_packets.pcap 'udp and (udp[8:4] = 0x52545053 or udp[12:4] = 0x52545053)' 2>/dev/null | wc -l)
  echo "Collected RTPS packets: $packet_count"
  rm rtps_packets.pcap
  exit
}
tcpdump -i any -s0 -n -v -X 'udp and (udp[8:4] = 0x52545053 or udp[12:4] = 0x52545053)' 2>/dev/null
