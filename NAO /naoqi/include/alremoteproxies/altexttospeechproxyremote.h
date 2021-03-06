// Generated for ALTextToSpeech version 1.14

#ifndef ALTEXTTOSPEECHPROXYREMOTE_H_
#define ALTEXTTOSPEECHPROXYREMOTE_H_
#include <alremoteproxies/altexttospeechproxyposthandlerremote.h>

namespace AL
{

/// <summary>This module embeds a speech synthetizer whose role is to convert text commands into sound waves that are then either sent to Nao's loudspeakers or written into a file. This service supports several languages and some parameters of the synthetizer can be tuned to change each language's synthetic voice.</summary>
class ALTextToSpeechProxyRemote : public ALProxyRemote
{
  public:

    /// <summary>
    /// Remote Constructor
    /// </summary>
    /// <param name="pIP"> The IP address used for the connection</param>
    /// <param name="pPort"> The port used for the connection </param>
    ALTextToSpeechProxyRemote(std::string pIP, int pPort) : ALProxyRemote("ALTextToSpeech", pIP, pPort)
    {
        post.setParent( (ALProxyRemote*)this );
    }

    /// <summary>
    /// Implements thread wrappers around methods
    /// </summary>
    ALTextToSpeechProxyPostHandlerRemote post;


    /// <summary>
    /// Disables the notifications puted in ALMemory during the synthesis (TextStarted, TextDone, CurrentBookMark, CurrentWord, ...)
    /// </summary>
    void disableNotifications()
    {

        callVoidRemote("disableNotifications");
    }


    /// <summary>
    /// Enables the notifications puted in ALMemory during the synthesis (TextStarted, TextDone, CurrentBookMark, CurrentWord, ...)
    /// </summary>
    void enableNotifications()
    {

        callVoidRemote("enableNotifications");
    }


    /// <summary>
    /// Exits and unregisters the module.
    /// </summary>
    void exit()
    {

        callVoidRemote("exit");
    }


    /// <summary>
    /// Outputs the languages installed on the system.
    /// </summary>
    /// <returns> Array of std::string that contains the languages installed on the system. </returns>
    std::vector<std::string> getAvailableLanguages()
    {

        return callRemote<std::vector<std::string> >("getAvailableLanguages");
    }


    /// <summary>
    /// Outputs the available voices. The returned list contains the voice IDs.
    /// </summary>
    /// <returns> Array of std::string containing the voices installed on the system. </returns>
    std::vector<std::string> getAvailableVoices()
    {

        return callRemote<std::vector<std::string> >("getAvailableVoices");
    }


    /// <summary>
    /// Gets the name of the parent broker.
    /// </summary>
    /// <returns> The name of the parent broker. </returns>
    std::string getBrokerName()
    {

        return callRemote<std::string >("getBrokerName");
    }


    /// <summary>
    /// Returns the language currently used by the text-to-speech engine.
    /// </summary>
    /// <returns> Language of the current voice. </returns>
    std::string getLanguage()
    {

        return callRemote<std::string >("getLanguage");
    }


    /// <summary>
    /// Returns the encoding that should be used with the specified language.
    /// </summary>
    /// <param name="pLanguage"> Language name (as a std::string). Must belong to the languages available in TTS. </param>
    /// <returns> Encoding of the specified language. </returns>
    std::string getLanguageEncoding(const std::string& pLanguage)
    {

        return callRemote<std::string >("getLanguageEncoding" , pLanguage);
    }


    /// <summary>
    /// Retrieves a method's description.
    /// </summary>
    /// <param name="methodName"> The name of the method. </param>
    /// <returns> A structure containing the method's description. </returns>
    AL::ALValue getMethodHelp(const std::string& methodName)
    {

        return callRemote<AL::ALValue >("getMethodHelp" , methodName);
    }


    /// <summary>
    /// Retrieves the module's method list.
    /// </summary>
    /// <returns> An array of method names. </returns>
    std::vector<std::string> getMethodList()
    {

        return callRemote<std::vector<std::string> >("getMethodList");
    }


    /// <summary>
    /// Retrieves the module's description.
    /// </summary>
    /// <returns> A structure describing the module. </returns>
    AL::ALValue getModuleHelp()
    {

        return callRemote<AL::ALValue >("getModuleHelp");
    }


    /// <summary>
    /// Returns the value of one of the voice parameters. The available parameters are: \"pitchShift\", \"doubleVoice\",\"doubleVoiceLevel\" and \"doubleVoiceTimeShift\"
    /// </summary>
    /// <param name="pParameterName"> Name of the parameter. </param>
    /// <returns> Value of the specified parameter </returns>
    float getParameter(const std::string& pParameterName)
    {

        return callRemote<float >("getParameter" , pParameterName);
    }


    /// <summary>
    /// Gets the method usage string. This summarises how to use the method.
    /// </summary>
    /// <param name="name"> The name of the method. </param>
    /// <returns> A string that summarises the usage of the method. </returns>
    std::string getUsage(const std::string& name)
    {

        return callRemote<std::string >("getUsage" , name);
    }


    /// <summary>
    /// Returns the voice currently used by the text-to-speech engine.
    /// </summary>
    /// <returns> Name of the current voice </returns>
    std::string getVoice()
    {

        return callRemote<std::string >("getVoice");
    }


    /// <summary>
    /// Fetches the current volume the text to speech.
    /// </summary>
    /// <returns> Volume (integer between 0 and 100). </returns>
    float getVolume()
    {

        return callRemote<float >("getVolume");
    }


    /// <summary>
    /// Returns true if the method is currently running.
    /// </summary>
    /// <param name="id"> The ID of the method that was returned when calling the method using 'post' </param>
    /// <returns> True if the method is currently running </returns>
    bool isRunning(const int& id)
    {

        return callRemote<bool >("isRunning" , id);
    }


    /// <summary>
    /// Loads a set of voice parameters defined in a xml file contained in the preferences folder.The name of the xml file must begin with ALTextToSpeech_Voice_
    /// </summary>
    /// <param name="pPreferenceName"> Name of the voice preference. </param>
    void loadVoicePreference(const std::string& pPreferenceName)
    {

        callVoidRemote("loadVoicePreference" , pPreferenceName);
    }


    /// <summary>
    /// Just a ping. Always returns true
    /// </summary>
    /// <returns> returns true </returns>
    bool ping()
    {

        return callRemote<bool >("ping");
    }


    /// <summary>
    /// Performs the text-to-speech operations : it takes a std::string as input and outputs a sound in both speakers. It logs an error if the std::string is empty. String encoding must be UTF8.
    /// </summary>
    /// <param name="stringToSay"> Text to say, encoded in UTF-8. </param>
    void say(const std::string& stringToSay)
    {

        callVoidRemote("say" , stringToSay);
    }


    /// <summary>
    /// Performs the text-to-speech operations: it takes a std::string as input and outputs the corresponding audio signal in the specified file.
    /// </summary>
    /// <param name="pStringToSay"> Text to say, encoded in UTF-8. </param>
    /// <param name="pFileName"> RAW file where to store the generated signal. The signal is encoded with a sample rate of 22050Hz, format S16_LE, 2 channels. </param>
    void sayToFile(const std::string& pStringToSay, const std::string& pFileName)
    {

        callVoidRemote("sayToFile" , pStringToSay, pFileName);
    }


    /// <summary>
    /// This method performs the text-to-speech operations: it takes a std::string, outputs the synthesis resulting audio signal in a file, and then plays the audio file. The file is deleted afterwards. It is useful when you want to perform a short synthesis, when few CPU is available. Do not use it if you want a low-latency synthesis or to synthesize a long std::string.
    /// </summary>
    /// <param name="pStringToSay"> Text to say, encoded in UTF-8. </param>
    void sayToFileAndPlay(const std::string& pStringToSay)
    {

        callVoidRemote("sayToFileAndPlay" , pStringToSay);
    }


    /// <summary>
    /// Changes the language used by the Text-to-Speech engine. It automatically changes the voice used since each of them is related to a unique language. If you want that change to take effect automatically after reboot of your robot, refer to the robot web page (setting page).
    /// </summary>
    /// <param name="pLanguage"> Language name. Must belong to the languages available in TTS (can be obtained with the getAvailableLanguages method).  It should be an identifier std::string. </param>
    void setLanguage(const std::string& pLanguage)
    {

        callVoidRemote("setLanguage" , pLanguage);
    }


    /// <summary>
    /// Changes the parameters of the voice. The available parameters are: 
    ///
    ///  	 pitchShift: applies a pitch shifting to the voice. The value indicates the ratio between the new fundamental frequencies and the old ones (examples: 2.0: an octave above, 1.5: a quint above). Correct range is (1.0 -- 4), or 0 to disable effect.
    ///
    ///  	 doubleVoice: adds a second voice to the first one. The value indicates the ratio between the second voice fundamental frequency and the first one. Correct range is (1.0 -- 4), or 0 to disable effect 
    ///
    ///  	 doubleVoiceLevel: the corresponding value is the level of the double voice (1.0: equal to the main voice one). Correct range is (0 -- 4). 
    ///
    ///  	 doubleVoiceTimeShift: the corresponding value is the delay between the double voice and the main one. Correct range is (0 -- 0.5) 
    ///
    ///  If the effect value is not available, the effect parameter remains unchanged.
    /// </summary>
    /// <param name="pEffectName"> Name of the parameter. </param>
    /// <param name="pEffectValue"> Value of the parameter. </param>
    void setParameter(const std::string& pEffectName, const float& pEffectValue)
    {

        callVoidRemote("setParameter" , pEffectName, pEffectValue);
    }


    /// <summary>
    /// Changes the voice used by the text-to-speech engine. The voice identifier must belong to the installed voices, that can be listed using the 'getAvailableVoices' method. If the voice is not available, it remains unchanged. No exception is thrown in this case. For the time being, only two voices are available by default : Kenny22Enhanced (English voice) and Julie22Enhanced (French voice)
    /// </summary>
    /// <param name="pVoiceID"> The voice (as a std::string). </param>
    void setVoice(const std::string& pVoiceID)
    {

        callVoidRemote("setVoice" , pVoiceID);
    }


    /// <summary>
    /// Sets the volume of text-to-speech output.
    /// </summary>
    /// <param name="volume"> Volume (between 0.0 and 1.0). </param>
    void setVolume(const float& volume)
    {

        callVoidRemote("setVolume" , volume);
    }


    /// <summary>
    /// returns true if the method is currently running
    /// </summary>
    /// <param name="id"> the ID of the method to wait for </param>
    void stop(const int& id)
    {

        callVoidRemote("stop" , id);
    }


    /// <summary>
    /// This method stops the current and all the pending tasks immediately.
    /// </summary>
    void stopAll()
    {

        callVoidRemote("stopAll");
    }


    /// <summary>
    /// Returns the version of the module.
    /// </summary>
    /// <returns> A string containing the version of the module. </returns>
    std::string version()
    {

        return callRemote<std::string >("version");
    }


    /// <summary>
    /// Wait for the end of a long running method that was called using 'post'
    /// </summary>
    /// <param name="id"> The ID of the method that was returned when calling the method using 'post' </param>
    /// <param name="timeoutPeriod"> The timeout period in ms. To wait indefinately, use a timeoutPeriod of zero. </param>
    /// <returns> True if the timeout period terminated. False if the method returned. </returns>
    bool wait(const int& id, const int& timeoutPeriod)
    {

        return callRemote<bool >("wait" , id, timeoutPeriod);
    }


};

}
#endif // ALTEXTTOSPEECHPROXY_H_
