clc
clear
close all

global CHORD_FUNDAMENTAL CHORD_QUALITY
CHORD_FUNDAMENTAL = {'C', 'C#', 'D', 'Eb', 'E', 'F', 'F#', 'G', 'Ab', 'A', 'Bb', 'B'};
CHORD_QUALITY.Minor = 0;
CHORD_QUALITY.Major = 1;
CHORD_QUALITY.Suspended = 2;
CHORD_QUALITY.Dominant = 3;
CHORD_QUALITY.Diminished5th = 4;
CHORD_QUALITY.Augmented5th = 5;

rosshutdown
rosinit

subChord = rossubscriber('/chords', 'std_msgs/Int8MultiArray');

while true
    msgChord = receive(subChord, 1);
    chordString = msgChord2string(msgChord);
    disp(['Detected chord: ', chordString])
end

function chordString = msgChord2string(msgChord)
global CHORD_FUNDAMENTAL CHORD_QUALITY
chordRootNote = msgChord.Data(1);
chordQuality = msgChord.Data(2);
chordIntervals = msgChord.Data(3);
switch chordQuality
    case CHORD_QUALITY.Major
        if chordIntervals == 0
            chordString = CHORD_FUNDAMENTAL{chordRootNote+1};
        else % == 7
            chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, 'maj7'];
        end
    case CHORD_QUALITY.Minor
        if chordIntervals == 0
            chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, '-'];
        else % == 7
            chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, '-7'];
        end
    case CHORD_QUALITY.Dominant
        chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, '7'];
    case CHORD_QUALITY.Suspended
        if chordIntervals == 2
            chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, 'sus2'];
        elseif chordIntervals
            chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, 'sus4'];
        end
    case CHORD_QUALITY.Diminished5th
        chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, 'dim'];
    case CHORD_QUALITY.Augmented5th
        chordString = [CHORD_FUNDAMENTAL{chordRootNote+1}, 'aug'];
end
end
