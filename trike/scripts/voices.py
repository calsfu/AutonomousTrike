import pyttsx3

engine = pyttsx3.init()
voices = engine.getProperty('voices')
for voice in voices:
    print(f"Voice ID: {voice.id}")
    print(f"  Name: {voice.name}")
    print(f"  Languages: {voice.languages}")
    print("---")