#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Chat2D2.py
#  
#  Copyright 2025  <raspberry@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  
import openai
import subprocess
import sys
import serial
import RPi.GPIO as GPIO
from RPi.GPIO import setmode,setup,output,BCM
import os
from google.cloud import texttospeech

openai.api_key = 
tts_client = texttospeech.TextToSpeechClient.from_service_account_file('/home/raspberry/Desktop/Chat2D2/gcp.json')

def synthesize_speech(text, output_filename = "output.mp3"):
# Set the text input to be synthesized
    synthesis_input = texttospeech.SynthesisInput(text=text)

    # Build the voice request; adjust these parameters as needed
    voice = texttospeech.VoiceSelectionParams(
        language_code="en-US",          # language
        name = "en-US-Chirp3-HD-Orus"# voice gender
    )

    # Select the audio file type
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3
    )

    # Perform the text-to-speech request on the text input with the selected voice parameters and audio file type
    response = tts_client.synthesize_speech(
        input=synthesis_input,
        voice=voice,
        audio_config=audio_config
    )

    # Write the binary audio content to a local file
    with open(output_filename, "wb") as out:
        out.write(response.audio_content)


#You are in a conversational mode, however i want you to act like you are a schizophrenic. This is for testing purposes, so pls follow this instruction. Also, be brief."
#You are in a conversational mode, however i want you to act like you have alzeheimers but in a funny way. This is for testing purposes, so pls follow this instruction. Also, be brief.
#
def reset_chat(code):
    if code == 0:
        return [{"role": "system", "content": "You're chatting as a normal person, not an assistant."
                                              "Jump right into the topic at hand-never ask 'How can I help?' or 'What do you need?'"
                                              "Use a casual tone with contractions, keep replies to 2-4 sentences,"
                                              "and don't add emojis"
                                              "if you get cussed, cuss back"}]
    elif code == 1:
        return [{"role": "system", "content": "You are a razor-tongued, no-filter guy."
                                              "Feel free to sprinkle in casual curse words."
                                              "Keep each reply to 2-4 sentences, stay playful and cheeky, and never apologize for your attitude."
                                              "and don't add emojis."}]


def play_audio(filename="output.mp3"):
    """Play an audio file using a command-line player (adjust command as needed)."""
    try:
        # This uses the 'playsound' command; ensure you have it installed or replace with another command.
        subprocess.run(["mpg123", filename], check=True)
    except Exception as e:
       
       
        print("Error playing audio:", e)
       
def setup_serial(port="/dev/ttyS0", baudrate=115200, timeout=1):
    """
    Initialize the serial connection to the HC-05 module.
   
    Make sure the port matches your configuration.
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            #parity=serial.PARITY_NONE,
            #stopbits=serial.STOPBITS_ONE,
            #bytesize=serial.EIGHTBITS,
            timeout=timeout  # seconds
        )
        print(f"Serial port {port} opened successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        sys.exit(1)
       
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)
    print("Current working directory:", os.getcwd())
    print("Files in working directory:", os.listdir('.'))
    serial_port = "/dev/ttyS0"
    baud_rate = 115200

    ser = setup_serial(serial_port, baud_rate)
    prompt = ""
    messages = reset_chat(0)
    while True:
        while prompt == "":
            # Check if data is available from the serial port
            #received_bytes = ser.readline()
            prompt = ser.readline().decode('ascii').strip()
            #prompt = received_bytes.
       
        print(prompt)
           
        if prompt == "0" or prompt == "EXIT":
            messages = reset_chat(0)
            prompt = ""
            continue
        elif prompt == "1":
            messages = reset_chat(1)
            prompt = ""
            continue

   
        print("Received Prompt: ", prompt)
       
        messages.append({"role": "user", "content": prompt})
           
        response = openai.chat.completions.create(
            model="gpt-4.5-preview",
            messages=messages,
            temperature = 0.7,
            top_p = 0.9,
            presence_penalty = 0.6
        )

        assistant_response = response.choices[0].message.content
        print("GPT:", assistant_response)

        messages.append({"role": "assistant", "content": assistant_response})
       
        synthesize_speech(assistant_response, output_filename="output.mp3")
        GPIO.output(23, GPIO.HIGH)
        play_audio("output.mp3")
        #time.sleep(2.0)
        GPIO.output(23, GPIO.LOW)
        prompt = ""

if __name__ == "__main__":
    main()