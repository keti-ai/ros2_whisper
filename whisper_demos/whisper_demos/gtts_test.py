from gtts import gTTS
import playsound as ps

text ="안녕하세요"

tts = gTTS(text=text, lang='ko')
tts.save("tmp_tts.mp3")
ps.playsound("tmp_tts.mp3")


text ="오늘의 날씨는"

tts = gTTS(text=text, lang='ko')
tts.save("tmp_tts.mp3")
ps.playsound("tmp_tts.mp3")

text ="매우 좋습니다 "

tts = gTTS(text=text, lang='ko')
tts.save("tmp_tts.mp3")
ps.playsound("tmp_tts.mp3")