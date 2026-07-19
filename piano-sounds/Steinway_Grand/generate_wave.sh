for f in *.mp3
	do ffmpeg -i "$f" "$(basename $f).wav"
done
