#!/usr/bin/env python3
"""
UDP Frame Receiver for ESP32 Camera Stream
Receives RGB565 frames over UDP, handles fragmentation, and displays them.
"""

import socket
import struct
import numpy as np
import cv2
import sys
from dataclasses import dataclass
from typing import Optional
import time

# Frame markers matching the ESP32 sender
FRAME_START_MARKER = bytes([0xFF, 0xF5])
FRAME_END_MARKER = bytes([0xFF, 0xF5, ord('E'), ord('D'), 0xF9])

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 3333
BUFFER_SIZE = 65536  # Maximum UDP packet size
CONNECTION_TIMEOUT = 5.0  # Seconds

@dataclass
class FrameState:
    """Tracks the state of frame reception"""
    receiving: bool = False
    expected_length: int = 0
    data: bytearray = None
    last_packet_time: float = 0.0
    
    def reset(self):
        """Reset frame state"""
        self.receiving = False
        self.expected_length = 0
        self.data = bytearray()
        self.last_packet_time = 0.0


class UDPFrameReceiver:
    """Handles UDP frame reception and decoding"""
    
    def __init__(self, ip: str = UDP_IP, port: int = UDP_PORT):
        self.ip = ip
        self.port = port
        self.sock = None
        self.frame_state = FrameState()
        self.frame_count = 0
        self.dropped_frames = 0
        self.total_bytes = 0
        
    def setup_socket(self):
        """Initialize UDP socket"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.ip, self.port))
            self.sock.settimeout(1.0)  # 1 second timeout for recv
            print(f"[INFO] UDP socket listening on {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to setup socket: {e}")
            return False
    
    def send_handshake(self, dest_addr):
        """Send handshake to ESP32 to start streaming"""
        try:
            self.sock.sendto(b"START", dest_addr)
            print(f"[INFO] Sent handshake to {dest_addr}")
        except Exception as e:
            print(f"[WARN] Failed to send handshake: {e}")
    
    def decode_rgb565_to_bgr(self, rgb565_data: bytes, width: int = 240, height: int = 240) -> Optional[np.ndarray]:
        """
        Decode RGB565 frame data to BGR format for OpenCV
        
        Args:
            rgb565_data: Raw RGB565 bytes
            width: Frame width
            height: Frame height
            
        Returns:
            BGR numpy array or None if decoding fails
        """
        try:
            expected_size = width * height * 2  # 2 bytes per pixel
            if len(rgb565_data) < expected_size:
                print(f"[WARN] Frame data too short: {len(rgb565_data)} < {expected_size}")
                return None
            
            # Convert bytes to uint16 array (RGB565 format)
            rgb565_array = np.frombuffer(rgb565_data[:expected_size], dtype=np.uint16)
            rgb565_array = rgb565_array.reshape((height, width))
            
            # Extract RGB components from RGB565
            # RGB565 format: RRRRR GGGGGG BBBBB
            r = ((rgb565_array & 0xF800) >> 11).astype(np.uint8)
            g = ((rgb565_array & 0x07E0) >> 5).astype(np.uint8)
            b = (rgb565_array & 0x001F).astype(np.uint8)
            
            # Scale to 8-bit (0-255)
            r = (r * 255 // 31).astype(np.uint8)
            g = (g * 255 // 63).astype(np.uint8)
            b = (b * 255 // 31).astype(np.uint8)
            
            # Combine into BGR format (OpenCV uses BGR)
            bgr_image = np.stack([b, g, r], axis=2)
            
            return bgr_image
            
        except Exception as e:
            print(f"[ERROR] Failed to decode RGB565: {e}")
            return None
    
    def process_packet(self, data: bytes) -> Optional[np.ndarray]:
        """
        Process incoming UDP packet and return complete frame if available
        
        Args:
            data: Raw packet data
            
        Returns:
            BGR image array if frame is complete, None otherwise
        """
        current_time = time.time()
        
        # Check for timeout (incomplete frame)
        if self.frame_state.receiving and \
           (current_time - self.frame_state.last_packet_time) > CONNECTION_TIMEOUT:
            print(f"[WARN] Frame timeout - dropping incomplete frame "
                  f"({len(self.frame_state.data)}/{self.frame_state.expected_length} bytes)")
            self.dropped_frames += 1
            self.frame_state.reset()
        
        # Check for start marker
        if data[:2] == FRAME_START_MARKER and len(data) >= 5:
            # Check if this is a start marker (not end marker)
            if data[4] == 0xF9 and data[2:4] != b'ED':
                # Extract frame length from marker
                frame_length = struct.unpack('>H', data[2:4])[0]
                
                if self.frame_state.receiving:
                    print(f"[WARN] New frame started before previous completed - dropping")
                    self.dropped_frames += 1
                
                self.frame_state.reset()
                self.frame_state.receiving = True
                self.frame_state.expected_length = frame_length
                self.frame_state.data = bytearray()
                self.frame_state.last_packet_time = current_time
                
                print(f"[INFO] Frame start - expecting {frame_length} bytes")
                return None
        
        # Check for end marker
        if data == FRAME_END_MARKER:
            if self.frame_state.receiving:
                received_length = len(self.frame_state.data)
                expected_length = self.frame_state.expected_length
                
                print(f"[INFO] Frame end - received {received_length} bytes "
                      f"(expected {expected_length})")
                
                # Decode and return frame
                if received_length > 0:
                    frame_data = bytes(self.frame_state.data)
                    self.frame_state.reset()
                    
                    # Decode RGB565 to BGR
                    bgr_frame = self.decode_rgb565_to_bgr(frame_data)
                    
                    if bgr_frame is not None:
                        self.frame_count += 1
                        self.total_bytes += received_length
                        return bgr_frame
                    else:
                        self.dropped_frames += 1
                else:
                    print("[WARN] Empty frame received")
                    self.dropped_frames += 1
                    self.frame_state.reset()
            else:
                print("[WARN] End marker received without start marker")
            
            return None
        
        # Accumulate frame data
        if self.frame_state.receiving:
            self.frame_state.data.extend(data)
            self.frame_state.last_packet_time = current_time
            
            received = len(self.frame_state.data)
            expected = self.frame_state.expected_length
            
            if received <= expected:
                progress = (received / expected) * 100 if expected > 0 else 0
                print(f"[DATA] Received {received}/{expected} bytes ({progress:.1f}%)", 
                      end='\r')
            else:
                print(f"[WARN] Received more data than expected: {received}/{expected} bytes")
        
        return None
    
    def print_stats(self):
        """Print reception statistics"""
        total = self.frame_count + self.dropped_frames
        if total > 0:
            success_rate = (self.frame_count / total) * 100
            avg_size = self.total_bytes / self.frame_count if self.frame_count > 0 else 0
            
            print(f"\n{'='*60}")
            print(f"Statistics:")
            print(f"  Frames received: {self.frame_count}")
            print(f"  Frames dropped:  {self.dropped_frames}")
            print(f"  Success rate:    {success_rate:.2f}%")
            print(f"  Average size:    {avg_size:.0f} bytes")
            print(f"  Total data:      {self.total_bytes / 1024:.2f} KB")
            print(f"{'='*60}\n")
    
    def run(self):
        """Main reception loop"""
        if not self.setup_socket():
            return
        
        print("[INFO] Waiting for frames... (Press 'q' to quit)")
        print("[INFO] Send any UDP packet to ESP32 to start streaming")
        
        # Create window
        cv2.namedWindow('ESP32 Camera Stream', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ESP32 Camera Stream', 480, 480)
        
        handshake_sent = False
        last_stats_time = time.time()
        
        try:
            while True:
                try:
                    # Receive UDP packet
                    data, addr = self.sock.recvfrom(BUFFER_SIZE)
                    
                    # Send handshake on first packet
                    if not handshake_sent:
                        self.send_handshake(addr)
                        handshake_sent = True
                    
                    # Process packet
                    frame = self.process_packet(data)
                    
                    # Display frame if complete
                    if frame is not None:
                        # Add info overlay
                        info_text = f"Frame #{self.frame_count} | {frame.shape[1]}x{frame.shape[0]}"
                        cv2.putText(frame, info_text, (10, 20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        cv2.imshow('ESP32 Camera Stream', frame)
                    
                    # Handle keyboard input
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("\n[INFO] Quit requested")
                        break
                    elif key == ord('s'):
                        # Save current frame
                        if frame is not None:
                            filename = f"frame_{self.frame_count}_{int(time.time())}.png"
                            cv2.imwrite(filename, frame)
                            print(f"\n[INFO] Saved frame to {filename}")
                    
                    # Print stats every 10 seconds
                    if time.time() - last_stats_time > 10:
                        self.print_stats()
                        last_stats_time = time.time()
                        
                except socket.timeout:
                    # No data received, just continue
                    pass
                except KeyboardInterrupt:
                    print("\n[INFO] Interrupted by user")
                    break
                
        finally:
            self.print_stats()
            if self.sock:
                self.sock.close()
            cv2.destroyAllWindows()
            print("[INFO] Cleanup complete")


def main():
    """Entry point"""
    print("="*60)
    print("ESP32 Camera UDP Frame Receiver")
    print("="*60)
    print()
    
    # Parse command line arguments
    port = UDP_PORT
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"[ERROR] Invalid port: {sys.argv[1]}")
            sys.exit(1)
    
    receiver = UDPFrameReceiver(port=port)
    receiver.run()


if __name__ == "__main__":
    main()
