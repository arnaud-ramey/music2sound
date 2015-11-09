# samples from http://theremin.music.uiowa.edu/MIS.html
rm *.ogg
for INWAV in "$@"
do
  TRIMWAV=${INWAV}.trim.wav
  OUTOGG=${INWAV}.ogg
  echo -e "$INWAV\t -> $OUTOGG"
  # sox file.wav output.wav trim [start time in seconds] [duration in seconds]
  # also increase volume
  #~ sox --volume 2.0 $INWAV $TRIMWAV trim 0 3

  #~ silence [−l] above-periods [duration threshold[d|%]
  #~         [below-periods duration threshold[d|%]]
  #~ −l indicates that below-periods duration length of audio should be left intact at the beginning of each period of silence
  #~ non-zero above-periods, it trims audio up until it finds non-silence
  #~ sox  --volume 2.0 $INWAV $TRIMWAV  silence 1 1 10% 1 2 1%
  sox  --volume 2.0 $INWAV $TRIMWAV  silence -l 1 .1 .5% 1 .2 .5% trim 0 3
  oggenc --quality 6 $TRIMWAV --output=$OUTOGG --quiet
  #~ play $INWAV
  #~ play $TRIMWAV
  #~ play $OUTOGG
  rm $TRIMWAV
done

### rename
rename 's/aiff.ogg/ogg/' *

### print size
find . -name "*.ogg" -ls | awk '{total += $7} END {printf("*.ogg: %.3g M\n",total/1024/1024)}'

#~ cp *.ogg /home/user/manager_ros/long_term_memory/long_term_memory/ettsSkill/piano
