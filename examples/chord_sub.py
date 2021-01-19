import rospy
from std_msgs.msg import Int8MultiArray

CHORD_FUNDAMENTAL = ['C', 'C#', 'D', 'Eb', 'E', 'F', 'F#', 'G', 'Ab', 'A', 'Bb', 'B'];
class CHORD_QUALITY:
    Minor = 0;
    Major = 1;
    Suspended = 2;
    Dominant = 3;
    Diminished5th = 4;
    Augmented5th = 5;

def msg_chord_2_string(msgChord):
    chordRootNote = msgChord.data[0]
    chordQuality = msgChord.data[1]
    chordIntervals = msgChord.data[2]
    if chordQuality == CHORD_QUALITY.Major:
        if chordIntervals == 0:
            chordString = CHORD_FUNDAMENTAL[chordRootNote]
        elif chordIntervals == 7:
            chordString = CHORD_FUNDAMENTAL[chordRootNote]+'maj7'
    elif chordQuality == CHORD_QUALITY.Minor:
        if chordIntervals == 0:
            chordString = CHORD_FUNDAMENTAL[chordRootNote]+'-'
        elif chordIntervals == 7:
            chordString = CHORD_FUNDAMENTAL[chordRootNote]+'-7'
    elif chordQuality == CHORD_QUALITY.Dominant:
        chordString = CHORD_FUNDAMENTAL[chordRootNote]+'7'
    elif chordQuality == CHORD_QUALITY.Suspended:
        if chordIntervals == 2:
            chordString = CHORD_FUNDAMENTAL[chordRootNote]+'sus2'
        elif chordIntervals == 4:
            chordString = CHORD_FUNDAMENTAL[chordRootNote]+'sus4'
    elif chordQuality == CHORD_QUALITY.Diminished5th:
        chordString = CHORD_FUNDAMENTAL[chordRootNote]+'dim'
    elif chordQuality == CHORD_QUALITY.Augmented5th:
        chordString = CHORD_FUNDAMENTAL[chordRootNote]+'aug'
    return chordString

def chordCallback(data):
    rospy.loginfo('Detected chord: %s', msg_chord_2_string(data))

rospy.init_node('chord_sub', anonymous=True)
rospy.Subscriber('/chords', Int8MultiArray, chordCallback)
rospy.spin()
