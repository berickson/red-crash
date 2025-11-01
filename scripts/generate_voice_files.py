#!/usr/bin/env python3
"""
Generate WAV files from a text file using Piper TTS.

Usage:
  # Generate files with defaults (speaker_id=65, reads soundboard_entries.csv)
  ./generate_voice_files.py
  
  # Generate files with specific voice
  ./generate_voice_files.py --speaker-id 4
  
  # Generate files with multiple voices
  ./generate_voice_files.py --speaker-ids 4 13 17 25
  
  # Specify different input file
  ./generate_voice_files.py --input /path/to/entries.csv
  
  # Specify different output directory
  ./generate_voice_files.py --output-dir /path/to/output
  
  # Adjust voice parameters
  ./generate_voice_files.py --speaker-id 65 --length-scale 1.0 --noise-scale 0.667

Input file format (CSV):
  number,button_name,"text to speak"
  1,X,"yes"
  2,Y,"no"
  
Output structure:
  output_dir/
    65/              # folder named after speaker_id
      1-X-yes.wav
      2-Y-no.wav
"""

import argparse
import csv
import os
import wave
import numpy as np
from pathlib import Path

try:
    from piper.voice import PiperVoice
    from piper.config import SynthesisConfig
except ImportError:
    print("Error: piper-tts not installed")
    print("Install with: pip install piper-tts")
    exit(1)


def sanitize_filename(text):
    """Convert text to safe filename (remove special characters)"""
    # Remove or replace characters that are problematic in filenames
    safe = text.replace('"', '').replace("'", '').replace('/', '-')
    safe = safe.replace('\\', '-').replace('?', '').replace('!', '')
    safe = safe.replace(':', '').replace(';', '').replace(',', '')
    safe = safe.strip()
    # Limit length
    if len(safe) > 50:
        safe = safe[:50]
    return safe


def load_piper_voice(model_path, config_path):
    """Load Piper TTS voice model"""
    print(f"Loading Piper TTS voice from {model_path}")
    try:
        voice = PiperVoice.load(model_path, config_path, use_cuda=False)
        print("Piper TTS voice loaded successfully")
        return voice
    except Exception as e:
        print(f"Failed to load Piper TTS voice: {e}")
        return None


def synthesize_to_wav(voice, text, output_path, speaker_id=65, length_scale=1.0, 
                      noise_scale=0.667, noise_w_scale=0.8):
    """Synthesize text to WAV file"""
    print(f"  Synthesizing: {text}")
    
    try:
        # Create synthesis config
        syn_config = SynthesisConfig(
            speaker_id=speaker_id,
            length_scale=length_scale,
            noise_scale=noise_scale,
            noise_w_scale=noise_w_scale,
        )
        
        # Synthesize audio to memory
        audio_chunks = []
        for audio_chunk in voice.synthesize(text, syn_config):
            audio_chunks.append(audio_chunk.audio_int16_array)
        
        if not audio_chunks:
            print(f"  ERROR: No audio generated for: {text}")
            return False
        
        # Concatenate all audio chunks
        full_audio = np.concatenate(audio_chunks)
        
        # Write to WAV file
        with wave.open(output_path, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(voice.config.sample_rate)
            wav_file.writeframes(full_audio.tobytes())
        
        print(f"  Created: {output_path}")
        return True
        
    except Exception as e:
        print(f"  ERROR during synthesis: {e}")
        return False


def read_entries(input_file):
    """Read entries from CSV file with format: number,button_name,"text" """
    entries = []
    try:
        with open(input_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 3:
                    try:
                        number = int(row[0])
                        button_name = row[1].strip()
                        text = row[2].strip()
                        entries.append((number, button_name, text))
                    except ValueError:
                        print(f"Warning: Skipping invalid row: {row}")
        return entries
    except FileNotFoundError:
        print(f"Error: Input file not found: {input_file}")
        return []
    except Exception as e:
        print(f"Error reading input file: {e}")
        return []


def main():
    parser = argparse.ArgumentParser(
        description='Generate WAV files from text file using Piper TTS',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--input', '-i', type=str, 
                       default='src/joy_soundboard_ros/soundboard_entries.csv',
                       help='Input CSV file (format: number,button_name,"text")')
    parser.add_argument('--output-dir', '-o', type=str, 
                       default='src/joy_soundboard_ros/sounds/voices',
                       help='Output directory (default: src/joy_soundboard_ros/sounds/voices)')
    parser.add_argument('--speaker-id', type=int, default=65,
                       help='Speaker ID to use (default: 65)')
    parser.add_argument('--speaker-ids', type=int, nargs='+',
                       help='Generate with multiple speaker IDs (overrides --speaker-id)')
    parser.add_argument('--length-scale', type=float, default=1.0,
                       help='Length scale (speed, default 1.0)')
    parser.add_argument('--noise-scale', type=float, default=0.667,
                       help='Noise scale (variability, default 0.667)')
    parser.add_argument('--noise-w-scale', type=float, default=0.8,
                       help='Noise w scale (default 0.8)')
    parser.add_argument('--model-dir', type=str, 
                       default='/root/ros2_ws/src/speech_ros/voices',
                       help='Directory containing Piper voice models')
    
    args = parser.parse_args()
    
    # Determine which speaker IDs to use
    if args.speaker_ids:
        speaker_ids = args.speaker_ids
    else:
        speaker_ids = [args.speaker_id]
    
    # Read input entries
    print(f"Reading entries from: {args.input}")
    entries = read_entries(args.input)
    if not entries:
        print("No entries found or error reading file")
        return 1
    print(f"Found {len(entries)} entries")
    
    # Load Piper voice model
    model_path = os.path.join(args.model_dir, 'en_US-libritts_r-medium.onnx')
    config_path = os.path.join(args.model_dir, 'en_US-libritts_r-medium.onnx.json')
    
    voice = load_piper_voice(model_path, config_path)
    if voice is None:
        return 1
    
    # Process each speaker ID
    for speaker_id in speaker_ids:
        print(f"\n=== Processing Speaker ID: {speaker_id} ===")
        
        # Create output directory for this speaker
        speaker_dir = os.path.join(args.output_dir, str(speaker_id))
        Path(speaker_dir).mkdir(parents=True, exist_ok=True)
        print(f"Output directory: {speaker_dir}")
        
        # Generate WAV file for each entry
        success_count = 0
        for number, button_name, text in entries:
            # Create filename: number-button_name-text.wav
            safe_text = sanitize_filename(text)
            filename = f"{number}-{button_name}-{safe_text}.wav"
            output_path = os.path.join(speaker_dir, filename)
            
            # Synthesize
            if synthesize_to_wav(voice, text, output_path, 
                                speaker_id=speaker_id,
                                length_scale=args.length_scale,
                                noise_scale=args.noise_scale,
                                noise_w_scale=args.noise_w_scale):
                success_count += 1
        
        print(f"Successfully generated {success_count}/{len(entries)} files")
    
    print(f"\n=== Complete ===")
    print(f"Output location: {os.path.abspath(args.output_dir)}")
    return 0


if __name__ == '__main__':
    exit(main())
