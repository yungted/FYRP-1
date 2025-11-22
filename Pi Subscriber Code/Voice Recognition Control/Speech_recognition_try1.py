import speech_recognition as sr  #Import the speech recognition library

#Create a Recognizer variable, which will handle converting speech to text
r = sr.Recognizer()

#Create a Microphone variable to use the Pi’s default microphone for input
mic = sr.Microphone()

#Calibrate the microphone to filter out background noise 
with mic as source:
    r.adjust_for_ambient_noise(source, duration=1)
    print("Calibrated, start speaking...")

#Main loop - keeps running until the user says "exit"
while True:
    with mic as source:
        print("Listening...")
        
        #Capture audio from the microphone
        audio = r.listen(source)

    try:
        #Send the captured audio to Google’s speech recognition API
        words = r.recognize_google(audio)

        #Print the recognized text
        print("You said:", words)

        #If the user says "exit", stop the program
        if words.lower() == "exit":
            print("Exiting program...")
            break

#Exception Handling
    except sr.UnknownValueError:
        #This error occurs when the speech is not understood (e.g., too noisy or unclear)
        print("Sorry, I could not understand that.")

    except sr.RequestError as e:
        #This error occurs if there’s an issue reaching Google’s API (e.g., no internet)
        print("Could not request results from Google: {e}")
