
Preparation

Following guidelines will help and protect your ears and your equipment as a beginner with Renardo.

Activate “SafetyNet”

SafetyNet is a SuperCollider quark that protects users from dangerous audio signals. Install the necessary module in SuperCollider with the following command line:

Quarks.install("SafetyNet")

Go with your cursor over the respective line and press Ctrl + Return (Cmd + Enter) to trigger the command

Hint: There is a graphical window version for installing Quark elements. Use the command line below for this:

Quarks.gui

Always start low

If you start with a new player, it is advised starting with lower volume. The synths or samples can appear unpredictably loud depend on changes in attributes. Furthermore, it will sound nicer in its entire experience, if an instrument comes in with increasing volume instead to overwhelm other sounds in the mix.

p1 >> pluck(amplify=0.1) ... p1 >> pluck(amplify=0.3)

Experiment with care

Be cautious while using attribute values when experimenting. For example a high octave can lead to nasty sounds, which not only can hurt your equipment of your machine, but can damage your ears while wearing headphones as well.

In traditional music theory, the octave of middle C is 3. However, it is 5 in Renardo.

oct=5

Bad example:

oct=60

To get all default values of basic attributes of a player use:

print(Player("pluck").info())


Introduction
What is live coding?

“Live Coding is a new direction in electronic music and video, and is getting somewhere interesting. Live Coders exposes and rewire the innards of software while it generates improvised music.” - toplap.org

    Interactive programming as an audio and/or visual art performance
    Using code to describe rules for an art piece
    Live notation/composition as performance
    Code can be changed and re-executed in real-time, while the program is running (compose music while performing)
    Takes computer language into a social environment, thus making coding to a social activity

Why using code?

    Classical music with notation on sheets is already a code to write musical pieces
    Pitch, duration, loudness in sheet music is a code, that can be read by musicians
    With Live coding, you can:
    Flexible describe rules
    Hack the code without an UI
    Interact with your composition, while it is playing
    Operate on the edge of liveness

What is Renardo?

    Renardo is a rebirth of FoxDot, after it has been depreciated. Big thanks to the developer Ryan Kirkbride from Leeds UK for his distribution to the live coding community!
    Renardo is a Python package that comes with its own IDE and a plugin for Pulsar called Pulsardo
    Renardo plays music by accessing any SynthDefs loaded onto a local SuperCollider server with some custom bits of syntax to boot
    SuperCollider is a programming language originally released in 1996 by James McCartney for real-time audio synthesis and algorithmic compositions, that runs underneath the Renardo environment
    Live coding with Python via Renardo offers accessible states through its reactive and dynamic objects
    Renardo focuses on musical patterns, not the digital signal processing (DSP), which is programmed by SuperCollider and controlled via OSC
    Renardo has a clean syntax, that is easy to read, so the code can be understood by an audience and traditional musicians without knowing Renardo or Programming


Start a little Python

As Renardo uses Python, we start with some Python code. To execute code in Renardo, make sure your text cursor is in the ‘block’ of code (sections of text not separated by blank lines) and press Ctrl+Return.

Enter the following line in the text part of the editor and press Ctrl+Enter (Cmd+Return) while the cursor is positioned over at the line.

2 + 2

The output of an executed code is displayed in the console in the lower window of the program. The console displays the line you entered. Use the Python function print() to display the result.

print(2 + 2)

Now we’re going to wrap the equation in a variable. We will use variables often. Write the 2 lines directly below each other so that it can be completely executed as a block:

a = 2 + 2

print(a)

Variables can also be combined:

a = 2
b = 3
c = a + b
print(c)

If you only want to execute one line within the block, move the cursor over the line and press Alt + Enter. Try some text in quotation marks:

print("Hello lively coder!")

The general philosophy of Renardo is to create “Player()”-objects as simply as possible, while taking keyword arguments that mirrors SuperCollider’s Pbind-SynthDef relationships and schedules their actions on a globally accessible clock.

If you want to know more about a function or class – just type help followed by the name of that Python object in brackets >> help(object), e.g.:

help(Player())

A SynthDef in Renardo is a Player() object. It is essentially your digital instrument you will use in your composition.


Synth Player
Synth Player Object

Renardo has a number of different virtual instruments available that you can use as player objects.

To have a look at the existing selection of Renardo SynthDefs, just execute:

print(SynthDefs)

Choose one and create a Renardo player object using the double arrow syntax like in the example below. In Renardo, all two-character variable names, such as p1, zz or k7, are reserved for Player() objects. The variable can consist of 2 letters or 1 letter + 1 number (e.g. pp or s1).

The >> in Python is usually reserved for a type of operation, like + or -, but it is not the case in Renardo.

In the following example the variable p1 will create a Player()-Object using pluck as synth/instrument. Creating a Player Object with a synthesizer and no arguments will play a single note on middle C, by default, repeatedly until stopped.

p1 >> pluck()

To stop an individual player object, simply execute p1.stop(). To stop all player objects, you can press CTRL+., which is a shortcut for the command Clock.clear().
Attribute degree

If you want to play musical notes, you need to give your player object some arguments. In this unique case of SynthDef you do not need to use an attribute name to change the notes being played back.

s1 >> pluck([0,2,4])

The “notes” we give to a player, meaning in this case the numbers 0, 2, and 4 are called degree. So the example above can be also look like this:

s1 >> pluck(degree=[0,2,4])

Use can use a TimeVar-Function to control the trigger of each note player over time. If you do not use any timing, it will be played each beat as default

s1 >> pluck(var([0,2,4]))

Just add your beat timing behind your note list, if you want to change timing

s1 >> pluck(var([0,2,4], 4))

Following allocation takes place:

Beat 0 –> Note 0 | Beat 4 –> Note 2 | Beat 8 –> Note 4 | Beat 12 –> Note 0 | Beat 16 –> Note 2 |…

To check which note is player at the moment, you cann use following code using degree within Pythons print() function:

print(s1.degree)

Create another list with numbers to change the time of each of the notes separately:

s1 >> pluck(var([0,2,4], [2,2,4]))

You can group notes and variables by enclosing multiple values of arguments as tuple in round brackets. This is often used to play chords. In the following example we play 2 notes at the same time and expand the stereo effect in the pan attribute:

p2 >> bass([(0,4),(0,2)], dur=4, pan=(-1,1))

It is possible to transfer a note played by one SynthDef to another. In this example s2 adds a triad to every bass note played by s1:

s1 >> bass([0,2,3,4], dur=4)

s2 >> pluck(dur=0.5).follow(s1) + (0,2,4)

In addition to .follow(), you can also use the .degree argument (without brackets) to follow other players:

s3 >> pluck(s1.degree + 2)

It is also possible to manipulate degree by adding an array of numbers to the Player object.

This raises the 4th note played by 2 degrees:

p1 >> pads([0,1,2,3]) + [0,0,0,2]

And this raises every third note by 2:

p1 >> pads([0,1,2,3]) + [0,0,2]

These values can be laced and grouped together

p1 >> pads([0,1,2,3]) + [0,1,[0,(0,2)]]

This behaviour is particularly useful when using the follow method.

b1 >> bass([0,4,5,3], dur=2)
p1 >> pads().follow(b1) + [2,4,7]

Next, you can schedule players to do things!

This will tell p1 to reverse the notes every 4 beats:

p1 >> pads([0,2,4,6])
p1.every(4, "reverse")

You can “chain” methods together by appending them to the end of the original line:

p1 >> pads([0,2,4,6]).every(4, "reverse")

To stop calling “reverse”, use ‘never’:

p1.never("reverse")

Here are a few other methods you can use!

Using “stutter” will play the same note n number of times with different attributes specified

p1.every(4, "stutter", 4, oct=4, pan=[-1,1])

Rotate will move all the values over by 1 in their order:

p1.every(4, "rotate")

To randomise the order of the notes, use “shuffle”:

p1.every(4, "shuffle")

Use other Attributes

Values assigned via named attributes shape the way the instrument sounds and is played. It is possible to use other arguments the same way like the examples above using degree. For example s1.oct, s1.dur and so on.

List all universal attributes:

print(Player.get_attributes())

List all attributes of a particular SynthDef:

print(Player("wobblebass").get_extra_attributes())

List all default fx attributes of Player() object:

print(Player.get_fxs())

You can see what effects are available by evaluating

print(FxList)

Let’s use the high pass filter for an example. You can see it’s described like so: “<Fx ‘highPassFilter’ — args: hpr, hpf>“

Each effect has a “master” argument and then child arguments. Here the master argument is “hpf” (short for high pass filter) and the child argument is “hpr” (short for high pass resonance). The effect is only added when the master argument is non-zero:

d1 >> dirt([0,4,2,1], dur=1/2, hpf=4000)

This sets the high pass filter to 4000 Hz so only frequences in the audio signal above that are actually heard. Let’s change the resonance value. It’s default value is 1, so let’s make it smaller:

d1 >> dirt([0,4,2,1], dur=1/2, hpf=4000, hpr=0.3)

Notice a difference? We can use patterns / vars in our effects to make them change over time:

d1 >> dirt([0,4,2,1], dur=1/2, hpf=linvar([0,4000],8), hpr=P[1,1,0.3].stretch(8))

In the following example octave oct is increased (Default is 5), the note playing time dur (Default is 1) and the volume amp varies (Default is 1).

Note: The standard octave in Renardo is 5, which in conventional music theory is 3!

s1 >> pluck([0,2,4], oct=6, dur=[1,0.5,0.5], amp=[1,0.75,0.75])

Arguments can be integers, floating points, fractions, lists, tuples, or a mix

p1 >> pluck([0,0,0], dur=2)
p1 >> pluck([0,0,0], dur=1.743)
p1 >> pluck([0,0,0], dur=[0.25,0.5,0.75])
p1 >> pluck([0,0,0], dur=[1/4,1/2,3/4])
p1 >> pluck([0,0,0], dur=[1/4,0.25,3])

You can also assign values to the attributes of player objects directly

p1 >> pluck([0,2], oct=5)

p1.oct = 4

Here some more useful attributes you can use in handling your players

Play only this player, muting others

p1.solo() # default value is 1 (solo on)

And turn the solo off

p1.solo(0)

Stop (not just mute) the other players

p1.only()

Referencing Attributes

You can set variables outside a player

pitches = P[0,1,2,3,4]
harmony = pitches + 2

print(pitches)
print(harmony)

p1 >> pluck(pitches)
p2 >> star(harmony)

If you set the duration of the second, it might not have the desired effect

p1 >> pluck(pitches)
p2 >> star(harmony, dur=1/2)

It is possible for one player object to play exactly what another player is. To have one player follow another, just use the follow method:

p1 >> pluck(pitches)
p2 >> star(dur=1/2).follow(p1) + 2

You can explicitly reference attributes such as pitch or duration too:

p2 >> star(p1.pitch) + 2  # this is the same as .follow(p1)

Works for other attributes too

p1 >> pluck(pitches)
p2 >> star(dur=p1.dur).follow(p1) + 2

You can reference, and test for the current value. The == returns a 1 if true and a 0 if false.

print(p1.degree)
print(p1.degree == 2)

This allows you to do conditionals like:

p1 >> pluck([0,1,2,3], amp=(p1.degree==1))
p1 >> pluck([0,1,2,3], amp=(p1.degree>1))

Or change it to a different amp by multiplying by 4:

p1 >> pluck([0,1,2,3], amp=(p1.degree==1)*4)

Chain multiple conditionals

p1 >> pluck([0,1,2,3], amp=(p1.degree==1)*4 + (p1.degree==2)*1)

Which is the same as

p1 >> pluck([0,1,2,3], amp=p1.degree.map({1:4, 2:1}))

Rests

Rests can be added by using a rest object in the dur array. The rest silences the note that would have been played. Without a rest, 5 notes (yes, a dur=1 would work, but lets be explicit to counterpoint the next example)

p1 >> pads([0,1,2,3,4], dur=[1,1,1,1,1])

With a rest … 4 notes and a rest, note “4” is silenced for 4 beats:

p1 >> pads([0,1,2,3,4], dur=[1,1,1,1,rest(4)])

Attributes Reference

amp - Amplitude (defaults to 1)

Sets the volume of the note/pattern

d1 >> play("*", dur=1/2, amp=1)

Half Volume

d1 >> play("*", dur=1/2, amp=.5)

Creating a pattern with amp

d1 >> play("*", dur=1/2, amp=[1,0,1,1,0])

amplify - Changes amp, by multiplying agasint the existing value (instead of overwritting)

Creating a pattern with amp

d1 >> play("*", dur=1/2, amp=[1,0,1,1,0])
d1 >> play("*", dur=1/2, amplify=[.5,1,0])

Set up a “drop” in the music (Plays at full volume for 28, then 0 for 4)

p1 >> blip([0,1,2,3], amplify=var([1,0],[28,4]))

bend

benddelay - See bend

bits - The bit depth, in number of bits, that the signal is reduced to; this is a value between 1 and 24 where other values are ignored. Use crush to set the amount of reduction to the bitrate (defaults to 8)

bitcrush - See bits

blur

bpf - Band Pass Filter

bpnoise - See bpf

bpr - See bpf

bpm

buf

channel

chop - ‘Chops’ the signal into chunks using a low frequency pulse wave over the sustain of a note.

coarse

comb delay - See echo

crush

cut - Cuts a duration

p1 >> pluck(P[:8], dur=1/2, cut=1/8)
p1 >> pluck(P[:8], dur=1/2, cut=1/4)
p1 >> pluck(P[:8], dur=1/2, cut=1/2)

cutoff

decay - See echo

degree - The degree of the note, or pitch, can be specified by keyword (also the first positional)

p1 >> blip(degree=[0,1,2,3])

Which is the same as:

p1 >> blip([0,1,2,3])

Only plays the “root” note of the chord

b1 >> bass(p1.degree[0])

delay - A duration of time to wait before sending the information to SuperCollider (defaults to 0)

Delays every 3 note by .1

p1 >> blip([0,1,2,3], delay=[0,0,0.1])

Delays every 3 note by .5

p1 >> blip([0,1,2,3], delay=[0,0,0.5])

Plays the note once for each different delays

p1 >> blip([0,1,2,3], delay=(0,0.1))
p1 >> blip([0,1,2,3], delay=(0,0.25))
p1 >> blip([0,1,2,3], delay=(0,.1,.2,.3))

dist

dur - Durations (defaults to 1 and 1/2 for the Sample Player)

echo - Title keyword: echo, Attribute keyword(s): decay - Sets the decay time for any echo effect in beats, works best on Sample Player (defaults to 0) - Multiplied against the sustain value

d1 >> play("x-o-", echo=0.1)
d1 >> play("x-o-", echo=0.5)
p1 >> pluck(P[:8], echo=.25)
p1 >> pluck(P[:8], echo=.5)
p1 >> pluck(P[:8], echo=.5, decay=.5)

env

fmod

formant

freq

hpf - High Pass Filter - Filters out all the frequencies below given value, removing lower freqencies

4000 hertz

p1 >> pluck(P[:8], dur=1/2, hpf=4000)

HPF is 0 for 4 beats, then 4000 for 4 beats

p1 >> pluck(P[:8], dur=1/2, hpf=var([0,4000],[4,4]))

Linear change on hpf from 0 take 4 beats to get to 4000, 4 beats back to 0

p1 >> pluck(P[:8], dur=1/2, hpf=linvar([0,4000],[4,4]))

Linear change on hpf from 0 take 8 beats to get to 4000, then reset back to 0

p1 >> pluck(P[:8], dur=1/2, hpf=linvar([0,4000],[8,0]))

With resonance change (default is 1)

p1 >> pluck(P[:8], dur=1/2, hpf=linvar([0,4000],[8,0]), hpr=.5)

With resonance change as a linvar

p1 >> pluck(P[:8], dur=1/2, hpf=linvar([0,4000],[8,0]), hpr=linvar([0.1,1],12))

hpr - See hpf

lpf - Low Pass Filter - Filters out all the frequencies above given value, removing higher freqencies

4000 hertz

p1 >> pluck(P[:8], dur=1/2, lpf=400)

With resonance change as a linvar

p1 >> pluck(P[:8], dur=1/2, lpf=linvar([500,4000],[8,0]), lpr=linvar([0.1,1],12))

lpr - See lpf

midinote

pan - Panning, where -1 is far left, 1 is far right (defaults to 0)

pitch - See degree

pshift

oct

rate - Variable keyword used for misc. changes to a signal. E.g. Playback rate of the Sample Player (defaults to 1)

room - Title keyword: room, Attribute keyword(s): mix

The room argument specifies the size of the room

d1 >> play("x-o-", room=0.5)

Mix is the dry/wet mix of reverb or how much the reverb is mixed with the source. 1 is all reverb, 0 is no reverb at all. (Default 0.1)

d1 >> play("x-o-", room=0.5, mix=.5)

reverb - See Room

sample - Special keyword for Sample Players; selects another audio file from the bank of samples for a sample character.

scale

shape

slide - Slide To - Slides’ the frequency value of a signal to freq * (slide+1) over the duration of a note (defaults to 0)

p1 >> pluck(P[:8], dur=1/2, slide=1)
p1 >> pluck(P[:8], dur=1/2, slide=12)
p1 >> pluck(P[:8], dur=1/2, slide=var([0,-1],[12,4]))

slidedelay

slidefrom

slider

spread

spin

striate

stutter

sus - Sustain (defaults to dur)

swell

vib - Vibrato - Title keyword: vib, Attribute keyword(s): Vibrato (defaults to 0)

p1 >> pluck(P[:8], dur=1/2, vib=12)

With child attribute, vibdepth (default 0.2)

p1 >> pluck(P[:8], dur=1/2, vib=12, vibdepth=0.5)

vibdepth - See vib
Try this!

    Use print(SynthDef) to see all available synthesizers and try them out.
    Create a small bass line with 1-8 notes, chords with 1-8 chords, and a small melody.
    Use some of the attributes: the octave variable oct=, the duration variable dur= and/or the amplitude gain value amplify= to get a better result!


Sample Player
Sample Player Object

Renardo can also be used to sequence and manipulate audio samples. To do this all you need to do is use the special play() Player() object. Unlike synthesizer Player() objects, the first argument to play should be a string of characters, not numbers. As a result, more information can be encoded in the character string than the character itself means. Each character relates to a range of audio files such as kicks, hi-hats, snares, and other sounds. Each audio file will be stored in a buffer in SuperCollider.

To view which character relates to which audio file, execute:

print(Samples)

There is a sound pack folder in Renardo called /samples/0_foxdot_default. This folder contains all characters named folders with samples. In order to use or create your own sample pack, you will need to name clone of the folder structure with top folder name like 1_my_samples, with path /samples/1_my_samples/. You can call samples from your own sample pack with the attribute spack:

b1 >> play("x", spack=1)

The simplest drum pattern for disco is:

b1 >> play("x-o-")

A character refers to a sound and whitespace is used for silence, so you can spread sounds out in time:

bd >> play("x  x  ")

You also can use dots instead of whitespace:

bd >> play("x..x..")

Different types of brackets add more information to one sequence. Put two or more characters in round brackets, the sound alternates with the new loop one after the other, thus lacing sound samples:

The following is the same as ”-------=“:

hh >> play("---(-=)")

Simple pattern example:

d1 >> play("(x-)(-x)o-")

Nested brackets for more variety:

d1 >> play("(x-)(-(xo))o-")

Putting characters in square brackets will play them all in the space of one beat, and will be played like one character, not simultaneous, but in quick succession

d1 >> play("x-o[-o]")
d1 >> play("x-o[---]")
d1 >> play("x-o[-----]")
d1 >> play("x-o[--------------]")

Play a triplet in the fourth beat:

d1 >> play("x-o[---]", dur=1)

and can be put in round brackets as if they were one character themselves.

d1 >> play("x[--]o(=[-o])")

Use square brackets in round brackets:

d1 >> play("(x-)(-[-x])o-")

Use round brackets in squared brackets:

b1 >> play("x-o[-(xo)]")

You can combine the brackets however you like: the following patterns are identical

d1 >> play("x-o(-[-o])")
d1 >> play("x-o[-(o )]")

Curly braces select a sample sound at random if you want more variety:

d1 >> play("x-o{-=[--][-o]}")

Angle brackets combine patterns to be play simultaneously:

d1 >> play("<X   ><-   ><#   ><V   >")
d1 >> play("<X   >< -  ><  # ><   V>")

Each character is mapped to a folder of sound files and you can select different samples by using the “sample” keyword argument:

d1 >> play("(x[--])xu[--]")
d1 >> play("(x[--])xu[--]", sample=1)
d1 >> play("(x[--])xu[--]", sample=2)

Change the sample for each beat:

d1 >> play("(x[--])xu[--]", sample=[1,2,3])

You can layer two patterns together - note the “P”, look at tutorial 4 for more information:

d1 >> play(P["x-o-"] & P[" **"])

And change effects applied to all the layered patterns at the same time:

d1 >> play(P["x-o-"] & P[" **"], room=0.5)

Example from the player tutorial, but with samples instead Conditionals…

d1 >> play("x[--]xu[--]x", sample=(d1.degree=="x"))

Or change it to sample bank 2 by multiplying:

d1 >> play("x[--]xu[--]x", sample=(d1.degree=="x")*2)

Chain multiple conditionals:

d1 >> play("x[--]xu[--]x", sample=(d1.degree=="x")*2 + (d1.degree=="-")*5)

Which is the same as:

d1 >> play("x[--]xu[--]x", sample=d1.degree.map({"x":2, "-":5}))

Attribute sample

Each character refers to a folder with same character. Folders with a letter as character contains 2 sub-folders namely upper and lower.

Those folders and sub-folders contain audio files, that can be called by play-Player() objects.

The audio files are arranged in alphabetical order. Use the sample attribute to select an audio file in this folder. Default is the first sample file in each folder, thus sample=0.

b1 >> play("x-o-", sample=1)

Like any other argument, sample can be a list (one at a time) or even a tuple (simultaneously) of values.

p1 >> play("x-o-", sample=[0,1,2])

p1 >> play("x-o-", sample=(0,3))

The example for a single character can be given within the character string itself by surrounding the character with a ”|” + the position number:

Play sample=2 for the letter ‘o’:

p1 >> play("x-|o2|-")

This will overwrite the specified value under sample:

p1 >> play("x-|o2|-", sample=3)

The syntax can contain any of the parentheses previously used for the character and numbers.

Change the sample number:

p1 >> play("x-|o(12)|-")

Change the sign:

p1 >> play("x-|(o*)2)|-")

Play several different samples in one step:

p1 >> play("x-|o[23]|-")

Play a random sample:

b1 >> play("x-|o{1[23]}|-")

If you decide to use several Player() objects to create e.g. a drum set, then it is recommended to use sample conventional, thus giving you a different way to change samples in time by using TimeVar() functions.

Clock.bpm=142
brks = [1]*28 + [0]*4
# SAMPLES
k1 >> play("A", sample=var([0,2], 64), dur=2, delay=[0,0.5], amplify=0.75*P[brks], amp=1)
k2 >> play("A", sample=1, dur=4, delay=[0,(0,0.5),0,(0,1.5)], pshift=var([0,1], 32), amplify=0.6*P[brks], amp=1)
k3 >> play("V", sample=[0,1,0,3], dur=2, delay=k1.delay, amplify=0.5*P[brks], amp=1)
s1 >> play("O", sample=var([0,2], [32,16]), dur=2, delay=1, room=0.66, mix=0.5, amplify=0.7*P[brks], amp=1)
s2 >> play("i", sample=var([0,1], 64), dur=2, delay=[1,1,1,(1,[1.5,1+0.75])], room=0.66, mix=0.33, amplify=0.7, amp=1)
h1 >> play(":", sample=var([0,1], 32), dur=1, delay=0.5, amplify=5/6, amp=1)
h2 >> play("-", sample=PRand([0,1,2],32), dur=0.5, rate=linvar([0.75,1], 8), amplify=0.6*P[brks]).every(16,"stutter",3)
h3 >> play("s", sample=1, dur=0.5, room=0.6, mix=0.33, amplify=[0.9,1.2], amp=1)
p1 >> play("y", sample=var([2,1,3], [28,4]), dur=1/2, delay=[0,0.25,0.5,0.75,0,0.5], rate=2, shape=0.6, room=0.5, mix=0.5, amplify=var([1,0.6], [1,3])*P[0.8,1.3], amp=1)
drumset = Group(k1,k2,k3,s1,s2,h1,h2,h3,p1)
drumset.amp=1

Layering sequences

You can also use < > signs to layer multiple sequences simultaneously. Let’s start with two separate sequences and then put them together in a single line of code.

Note: The *dot is equivalent to space. Like space, it is a placeholder that helps to better recognize temporal positioning

b1 >> play("x-o-")

b2 >> play("..+.+.[.+]")

We can place any sequence between ”<>” characters in a single sequence and have them play at the same time:

b1 >> play("<x-o-><..+.+.[.+]>")

This is equivalent to:

b1 >> play(P["x-o-"].zip(P["..+.+.[.+]"]))

Zip can be understood as a zipper.

Each layer relates to the index in a group of values given to a Player()-object, each layer is affected only by one of those given values. This is best demonstrated by an example:

Pan each sequence hard on the left and right channels using square brackets in the pan attribute:

b1 >> play("<x-o-><..+.+.[.+]>", pan=[-1,1])

Expand the stereo effect by using round brackets:

b1 >> play("<x-o-><..+.+.[.+]>", pan=(-1,1))

Change the audio file used in the first layer:

b1 >> play("<x-o-><..+.+.[.+]>", sample=(2,0))

Be careful when combining multiple layers with functions like offadd as this functions create new layers.

The following code will only affect the second layer, so the first layer is unaffected:

b1 >> play("<x-o-><..+.+.[.+]>", sample=(2,0)).every(4, "sample.offadd", 2)

Try this!

Go through the characters and listen to the different examples available. Use the attribute sample=[:8]. The audio files or samples will be repeated if the character contains fewer than 9 samples (0-8 are 9 numbers) in the dedicated folder!
Name	Letter/Character
Kick	A v V x X W
Snare/Rim	D i I o O t u
Hihat	: = - a n N
Clap/Snap	\ * h H
Cymbal/Crash	/ # e E
Tom/Tom-like	m M p P w
Percussion	& + d f l r R y
SoundFX	\ b F k L Q Y z Z
Voice	1 2 3 4 ! < ? c C
Bell	T
Various	$ ; B g G j J K q U
Noise	@ %
Shaker	s S
Ride	~

Create a 16 beat rhythm with your preferred samples. Use Clock.bpm=120 to change the beat per minutes, or speed of rhythm in time!


Loop Player
Loop Player Object

You can use your own samples by simply dropping audio files into the existing FoxDot sample directories. These are found in the snd directory in the root of the Renardo installation (e.g., ‘/home/user/.config/renardo/samples/’).

You saw earlier how to work with samples using play(). The loop Player() object is similar to play. However, it plays an audio file by a given place given by a string containing “absolute_path/file_name” together, instead of using a sample file of an installed sample pack of Renardo.

You can also play samples with loop().

s1 >> loop('foxdot')

You may notice that this is just playing the first part of the sample over and over again. You can tweak the behavior with many of the arguments we’ve seen thus far for controlling other synths. dur is a good place to start.

s1 >> loop('foxdot', dur=4)

If you have a folder full of samples that you would like to use in FoxDot, you can call loop() with the full path to the sample.

s1 >> loop('/path/to/samples/quack.wav')

If you give loop the path to a folder, it will play the first sample it finds. You can change which sample it plays with the sample= arg.

Play the first sample in my collection

s1 >> loop('/path/to/samples')

Play the second sample in my collection

s1 >> loop('/path/to/samples', sample=1)

If you’re going to be using a lot of samples from a folder, you can add it to the sample search path. FoxDot will look under all its search paths for a matching sample when you give it a name.

Samples.addPath('/path/to/samples')
s1 >> loop('quack')

Once you have a search path, you can use pattern matching to search for samples. Play the 3rd sample under the ‘snare’ dir:

s1 >> loop('snare/*', sample=2)

You can use * in directory names too:

s1 >> loop('*_120bpm/drum*/kick*')

** means “all recursive subdirectories”. This will play the first sample nested under ‘percussion’ (e.g. ‘percussion/kicks/classic/808.wav’)

s1 >> loop('percussion/**/*')

You can put files in a special folder located in “/snd/loop” which can be opened by going to “Help & Settings” and then “Open Samples Folder” from the FoxDot editor menu. You don’t need to supply the full path (or extension) for files in this folder:

l1 >> loop("my_file", dur=4)

To see all the files in this folder use print(Samples.loops). If you want to play with the playback order, you can supply a “position” argument after the file name that Renardo will iterate through based on the duration.

Play first 4 beats of audio in order:

l1 >> loop("my_file", P[:4], dur=1)

Play first beats in random order:

l1 >> loop("my_file", P[:4].shuffle(), dur=1)

If you know the bpm of the audio file and wish to play it at the current tempo, you can supply the player with a tempo argument. For example, my_file could be a drum beat at 135 bpm but the current tempo is 120, I can fit the tempo of my_file to the clock like so:

First 4 beats in 1 beat steps:

l1 >> loop("my_file", P[:4], dur=1, tempo=135)

First 4 beats in 0.5 beat steps:

l1 >> loop("my_file", P[:8]/2, dur=0.5, tempo=135)

Time stretching

Time stretching the audio in this fashion will change the pitch. If the audio is pitched, you may wish the time-stretch it without losing that information. This is possible using the striate. This cuts the file into lots of little segments and plays them back spread out over the course of the duration value – this will play the entire audio file. The larger the audio file, the larger the number you will probably want to use. Using the example above, you may want to use a striate value of 100-200 for a smoother playback:

Stretch the file using 100 segments:

l1 >> loop("my_file", dur=4, striate=100)`

Stretch it using 10 segments - listen to the difference:

l1 >> loop("my_file", dur=4, striate=10)

An extra attribute for loop is beat_stretch=True, which will stretch the audio file length into its given duration.
Try This!

Search under www.wavsource.com or www.findsounds.com for 2-3 short audio files. Voices, vocals, beat loops, instruments or ambient noise are best.

The loop synth is designed to let you play longer audio files (>1 sec) and manipulate them. To get started, just supply the filename you want to play and the duration you want to play in beats:

l1 >> loop("path/to/my/file.wav", dur=32, sus=32)


Clock
Basics

To stop all player objects, you can press Ctrl+. (Hold Ctrl and hit the period). Which is a shortcut for the command:

Clock.clear()

Change the tempo (this takes effect at the next bar) Default is 120.

Clock.bpm = 144

To see what is scheduled to be played.

print(Clock)

To see what the latency is

print(Clock.latency)

Sometimes you want to know when the start of the next X beat cycle. To do this we use the ‘mod’ method. For example if we want to see when the start of the next 32 beat cycle is we can do

print(Clock.mod(32))

Advanced

The clock can schedule anything with a call method using. It takes an absolute time clue to schedule a functions - Clock.schedule needs to know the beat to call something on.

Clock.schedule()   # raises TypeError

Schedule an event after a certain durations - Clock.future needs to know how many beats ahead to call something

Clock.future()     # raises TypeError

These are equivalent

Clock.schedule(lambda: print("hello"), Clock.now() + 4)
Clock.future(4, lambda: print("hello"))

To schedule something else

Clock.schedule(lambda: print("hello "))

We can call something every n beats

Clock.every(4, lambda: print("hello"))

Get the current clock and add 2. - Useful for scheduling.

print(Clock.now() + 2)

Issue command on the next bar

nextBar(Clock.clear)

With a decorator

@nextBar
def change():
    Root.default=4
    Scale.default="minor"
    # etc etc

You can create your own function, and decorate it, to be able to use it in an .every on a Player object

@PlayerMethod
def test(self):
    print(self.degree)

p1 >> pluck([0,4]).every(3, "test")

And cancel it with

p1.never("test")


Patterns
Pattern

Renardo uses in its Player() objects Python lists, known more commonly as arrays in other languages, to sequence themselves. It has been used here already previously but they aren’t exactly flexible for manipulation.

For example, try multiplying a list by two like this:

print([1,2,3] * 2)

Console output >> [1,2,3,1,2,3]

Does the result meet your expectations?

Renardo uses a container type called a ‘Pattern’ to help solve this problem. They act like regular lists but any mathematical operation performed on it is done to each item in the list and done so pair-wise if using a second pattern. A basic pattern is created as you would with a normal list or tuple, but with a ‘P’ preceeding it.

print(P[1,2,3] * 2)
print(P[1,2,3] + 100)

In this operation, the output consists of all the combinations of the two patterns i.e. [1+3, 2+4, 3+3, 1+4, 2+3, 3+4]

print(P[1,2,3] + [3,4])

You can use Python’s slicing syntax to generate a series of numbers:

print(P[:8])
print(P[0,1,2,3:20])
print(P[2:15:3])

Try some other mathematical operators and see what results you get.

print(P[1,2,3] * (1,2))

Pattern objects also automatically interlace any nested list. Compare a normal list:

for n in [0,1,2,[3,4],5]:
    print(n)

with Pattern

for n in P[0,1,2,[3,4],5]:
    print(n)

Use PGroups if you want this behavior to be avoided. These can be implicitly specified as tuples in Patterns:

for n in P[0,1,2,(3,4)]:
    print(n)

This is a PGroup:

print(P(0,2,4) + 2)
print(type(P(0,2,4) + 2))

In Python, you can generate a range of integers with the syntax range(start, stop, step). By default, start is 0 and step is 1.

print(list(range(10))) # [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

You can use PRange(start, stop, step) to create a Pattern object with the equivalent values:

print(PRange(10)) # P[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

P[0, 2, 2, 6, 4, 10, 6, 14, 8, 18] [01, 12, 21, 32, 41, 52, 61, 72, 8*1…]

print(PRange(10) * [1, 2]) # Pattern class behaviour

Adding a list (or Pattern) to a Pattern will add the values of the elements to the other where Python lists would be concatonated.

print(PRange(10) + [0,10])

To concatonate Patterns, use the pipe operator like so:

print(PRange(10) | [0,10])

FoxDot automatically converts any object being piped to a Pattern to the base Pattern class so you don’t have to worry about making sure everything is the right type. Plays all the values together:

p1 >> pluck(P(4,6,8))
p1 >> pluck(P[0,1,2,P(4,6,8),7,8])

Spreads the values across the current “dur” e.g. if the dur is 2 beats then it will play each value 2/3 beats apart:

p1 >> pluck(P*(0,2,4), dur=1/2)
p1 >> pluck(P*(0,2,4), dur=1)
p1 >> pluck(P*(0,2,4), dur=2)
p1 >> pluck(P[0,1,2,P*(4,6,8),7,8], dur=1)

Is the same as P* but every other time the notes are played they are spread over the dur value.

p1 >> pluck(P/(0,2,4), dur=1/2)
p1 >> pluck(P/(0,2,4), dur=1)
p1 >> pluck(P/(0,2,4), dur=2)
p1 >> pluck(P[0,1,2,P/(4,6,8),7,8], dur=1)

Spreads the values across the current “sus” e.g. if the dur is 2 beats and the sus is 3 beats then it will play each value 1 beat apart.

p1 >> pluck(P+(0,2,4), dur=2, sus=3)
p1 >> pluck(P+(0,2,4), dur=2, sus=1)
p1 >> pluck(P[0,1,2,P+(4,6,8),7,8], dur=1, sus=3)

Spreads the first (length-1) values with a gap of the last value between each Plays 0,2,4 with a gap of 0.5:

p1 >> pluck(P^(0,2,4,0.5), dur=1/2)

Patterns come with several methods for manipulating the contents

help(Pattern)

Standard pattern

print(P[:8])

Shuffle pattern by randomizing it

print(P[:8].shuffle())

Append a reversed pattern to the pattern

print(P[:8].palindrome())

Shift the pattern by n (default 1)

print(P[:8].rotate())
print(P[:8].rotate(3))
print(P[:8].rotate(-3))

Takes the pattern and appends it as many times as needed to reach n number of elements in the pattern:

print(P[:8].stretch(12))
print(P[:8].stretch(20))

Reverses a pattern

print(P[:8].reverse())

Loops a pattern n number of times

print(P[:8].loop(2))

Add an offset

print(P[:8].offadd(5))

Add a multiplied offset

print(P[:8].offmul(5))

Stutter - Repeat each element n times

print(P[:8].stutter(5))

Amen - Merges and laces the first and last two items such that a drum pattern “x-o-” would become “(x[xo])-o([-o]-)” and mimics the rhythm of the famous “amen break”

d1 >> play(P["x-o-"].amen())
print(P[:8].amen())

Bubble - Merges and laces the first and last two items such that a drum pattern “x-o-” would become “(x[xo])-o([-o]-)

d1 >> play(P["x-o-"].bubble())
print(P[:8].bubble())

If you want to edit the internal values in Python you need to use a for loop:

l = []
for i in [1,2,3]:
    l.append(i*2)
    print(l)

or in the list understanding:

print([i*2 for i in [1,2,3]])

Console output >> [2,4,6]

But what if you want to multiply the values in a list by 2 and 3 alternately?

Renardo uses a type of container called “Pattern” to solve this problem. They behave like regular lists, but any math operation performed on them is performed on each item in the list, and paired if a second pattern is used.

The basic pattern can be created as follows:

print(P[1,2,3]*2)

Console output >> P[2,4,6]

print(P[1,2,3]+[3,4])

Console output >> P[4,6,6,5,5,7]

Notice how in the second operation the output is any combination of the two patterns >> [1+3,2+4,3+3,1+4,2+3,3+4].
Pattern

Try some other math operators and see what results you get!

What if you group numbers in brackets like P[1,2,3] * (1,2)?

P[P(1,2),P(2,4),P(3,6)]

There are several other pattern classes in Renardo that you can use to generate arrays of numbers, but they behave just like the base pattern.

print(classes(Patterns.Sequences))

print(classes(Patterns))

In Python you can use the syntax area (start, stop, step) to generate a range of integers. By default, Start is 0 and Step 1.

With PRange (start,stop,step) you can create a sample object with the appropriate values. The first example shows the equivalent function in Python, the second is the simplified sample function in Renardo PRange:

print(list(range(10)))

Console output >> [0,1,2,3,4,5,6,7,8,9]

print(PRange(10))

Console output >> P[0,1,2,3,4,5,6,7,8,9]

print(PRange(10)*[1,2])

Console output >> P[0,2,2,6,4,10,6,14,8,18]

But what about combining patterns? In Python you can concatenate (append) two lists with the + operator. However, Renardo patterns use this to supplement the data in the list. To connect two Pattern objects together, you can use the pipe symbol, which Linux users may be familiar with. It is used to connect command line programs by sending the output of one process as input to another.

print(PRange(4)|[1,7,6])

Console output >> P[0,1,2,3,1,7,6]

There are several types of pattern sequences in Renardo (and the list is still growing) that make generating these numbers a little easier. For example, to play the first octave of a pentatonic scale from bottom to top and back again, you can use two PRange objects:

p1 >> pluck(PRange(5)|PRange(5,0,-1), scale=Scale.default.pentatonic)

The PTri class does this for you:

p1 >> pluck(PTri(5), scale=Scale.default.pentatonic)

Pattern functions

There are several functions that generate a pattern of values for us to do useful things in Renardo, such as: Rhythms and melodies. This section is a list of pattern functions with descriptions and examples.

Used as input arguments for Renardo players, these can themselves be treated as patterns and their methods applied directly, e.g. B. PDur(3,8).reverse(). You can also replace each input argument with a pattern or a TimeVar function to create an advanced pattern or a Pvar pattern. Let’s look at some examples:

PStep(n,value,default=0) >> Returns a pattern where every n-term is value, otherwise default.

Every 4, make it 1, otherwise default to 0

print(PStep(4,1))

Every 8, make it 6, otherwise, 4

print(PStep(8,6,4))

Every 5, make it 2, otherwise, 1

print(PStep(5,2,1))

PSum(n,total,**kwargs) >> Returns a pattern of length n, the sum of which is total. For example: PSum(3,8) -> P[3,3,2] PSum(5,4) -> P[1,0.75,0.75,0.75,0.75].

Returns a pattern of length 2, with elements summed up to 8

print(PSum(3,8))

Returns a pattern of length 5, with elements summed up to 4

print(PSum(5,4))

PRange(start,stop=None,step=None) >> Returns a pattern equivalent to Pattern(range(start,stop,step)).

PTri(start,stop=None,step=None) >> Returns a pattern equivalent to Pattern(range(start,stop,step)) with the inverted shape appended. Think of it like a “Tri”angle.

Up to 5 then down to 1:

print(PTri(5))

Up to 8 then down to 1:

print(PTri(8))

From 3 to 10, then down to 4:

print(PTri(3,10))

From 3 to 30, by 2, then down to 4:

print(PTri(3,20,2))

Up to 4, then down to 1, then up to 8, then down to 1:

print(PTri([4,8]))
p1 >> pluck(PTri(5), scale=Scale.default.pentatonic)

Same as:

p1 >> pluck(PRange(5) | PRange(5,0,-1), scale=Scale.default.pentatonic)

PEuclid(n,k) >> Returns the Euclidean rhythm that distributes n pulses as evenly as possible over k steps. e.g. PEuclid(3,8) returns P[1,0,0,1,0,0,1,0]. 3 pulses over 8 steps:

print(PEuclid(3,8))

PSine(n=16) >> Returns values of one cycle of a sine wave divided into n parts.

Split into 5 parts:

print(PSine(5))

Split into 10:

print(PSine(10))

PDur(n,k,dur=0.25) >> Returns the actual duration based on Euclidean rhythms (see PEuclid), where dur is the length of each step. e.g. PDur(3,8) returns P[0.75,0.75,0.5].

print(PDur(3,8)) # P[0.75, 0.75, 0.5]
print(PDur(5,8))

Gives a list of 3 dur, appened with a list of 5 dur

print(PDur([3,5],8))
d1 >> play("x", dur=PDur(5,8))

PBern(size=16,ratio=0.5) >> Returns a pattern of ones and zeros based on the ratio value (between 0 and 1). This is known as the Bernoulli sequence.

PBeat(string,start=0,dur=0.5) >> Returns a pattern of durations based on an input string, where non-spaces denote a pulse.

PSq(a=1,b=2,c=3)

PIndex >> Returns the index being accessed

print(PIndex())
print(PIndex()*4)

Pattern generators

We know that patterns have a fixed length and can be generated based on a function. However, sometimes it is useful to have patterns of infinite length, for example when generating random numbers. This is where pattern generators come into play. Similar to Python generators where not all values are kept in memory at once, except when Python generators usually have an end - Renardo pattern generators don’t!

PRand(lo,hi,seed=None)/PRand([values]) >> Returns a series of random integers between lo and hi, inclusive. If hi is omitted, the range is 0 to lo. A list of values can be provided in place of the range and PRand returns a series of values chosen at random from the list.

Returns a random integer between 0 and start.

print(PRand(8)[:5])

Returns a random integer between start and stop.

print(PRand(8,16)[:5])

If start is a container-type it returns a random item for that container.

print(PRand([1,2,3])[:5])

You can supply a seed

print(PRand([1,2,3], seed=5)[:5])

Keeps generating random tune

p1 >> pluck(PRand(8))

Creates a random list, and iterates over that same list

p1 >> pluck(PRand(8)[:3])

PxRand(lo, hi) / PxRand([values]) >> Identical to PRand, but no elements are repeated.

PwRand([values], [weights]) >> Uses a list of weights to indicate how often items with the same index are selected from the list of values. A weight of 2 means it is twice as likely to be picked as an item weighing 1.

P10(n)>> Returns a pattern of length n of a randomly generated series of ones and zeros.

PAlt(pat1, pat2, *patN) >> Returns a pattern generated by alternating the values in the specified sequences.

PJoin(patterns) >> Assembles a list of patterns.

PPairs(seq,func=) >> Links a sequence to a second sequence obtained by executing a function on the original. By default, this is lambda n: 8-n.

PQuicken(dur=0.5,stepsize=3,steps=6) >> Returns a group of delay amounts that gradually decrease.

PRhythm(durations) >> Converts all tuples / PGroups into delays, which are calculated with the PDur algorithm.

The following plays the hi hat with a Euclidean Rhythm of 3 pulses in 8 steps

d1 >> play("x-o-", dur=PRhythm([2,(3,8)]))
print(PRhythm([2,(3,8)]))

PShuf(seq) >> Returns a mixed version of seq. This example uses a function to automatically shuffle the list.

PStretch(seq,size) >> Returns ‘seq’ as a pattern and is looped until its length is ‘size’, e.g. PStretch ([0,1,2], 5) returns P[0,1,2,0,1].

PStrum(n=4)

PStutter(seq,n=2) >> Creates a pattern so that each element in the array is repeated n times (n can be a pattern).

PZip(pat1,pat2, patN) >> Generates a pattern that ‘zips’ multiple patterns. PZip([0,1,2], [3,4]) creates the pattern P[(0,3),(1,4),(2,3),(0,4),(1,3),(2,4)].

PZip2(pat1,pat2,rule=) >> Like PZip, but only uses two patterns. Connects values if they meet the rule.

Pvar >> TimeVar, which saves lists instead of individual values (var, sinvar, linvar, expvar).

PWhite(lo,hi) >> Returns random floating point numbers between lo and hi.

Lo defaults to 0, hi defaults to 1

print(PWhite()[:8])

Returns random numbers between 1 and 5

print(PWhite(1,5)[:8])

PChain(mapping_dictionary) >> Based on a simple Markov chain with equal probabilities. Takes a dictionary of elements, states, and possible future states. Every future state has an equal chance of being selected. If a possible future state is not valid, a KeyError is raised.

PWalk(max=7,step=1,start=0) >> Returns a series of integers with each element an increment apart and with a value in the range of +/- the maximum. The first element can be selected with start.

By default, returns a pattern with each element randomly 1 higher or lower than the previous

print(PWalk()[:16])

Changing step

print(PWalk(step=2)[:16])

With max

print(PWalk(max=2)[:16])

Start at a non-zero number

print(PWalk(start=6)[:16])

PFibMod() >> Returns the Fibonacci sequence.
Custom Pattern Generator

Custom generator patterns can be made by subclassing GeneratorPattern and overriding GeneratorPattern.func

class CustomGeneratorPattern(GeneratorPattern):
    def func(self, index):
        return int(index / 4)
print(CustomGeneratorPattern()[:10])

This can be done more consisely using GeneratorPattern.from_func, passing in a function which takes an index and returns some pattern item.

def some_func(index):
    return int(index / 4)
print(GeneratorPattern.from_func(some_func)[:10])

We can use lambdas too

print(GeneratorPattern.from_func(lambda index: int(index / 4))[:10])


TimeVars
TimeVar var()

A TimeVar is an abbreviation of “Time Dependent Variable” and is a key feature of Renardo. A TimeVar has a series of values that it changes between after a pre-defined number of beats and is created using a var object with the syntax var([list_of_values], [list_of_durations]).

Generates the values: 0,0,0,0,3,3,3,3…

a = var([0,3],4)            # Duration can be single value
print(int(Clock.now()), a)  # 'a' initally has a value of 0

Console Output - (The first value may differ): 0, 0

print(int(Clock.now()), a)   # After 4 beats, the value changes to 3

Console Output: 4, 3

print(int(Clock.now()), a)   # After another 4 beats, the value changes to 0

Console Output: 8, 0

Duration can also be a list

a = var([0,3],[4,2])
print(int(Clock.now()), a)

When a TimeVar is used in a mathematical operation, the values it affects also become TimeVars that change state when the original TimeVar changes state – this can even be used with patterns:

a = var([0,3], 4)
print(int(Clock.now()), a + 5)   # When beat is 0, a is 5

Console Output: 5

print(int(Clock.now()), a + 5)   # When beat is 4, a is 8

Console Output: 8

b = PRange(4) + a
print(int(Clock.now()), b)   # After 8 beats, the value changes to 0

Console Output: P[0, 1, 2, 3]

print(int(Clock.now()), b)   # After 12 beats, the value changes to 3

Console Output: P[3, 4, 5, 6]

Use ‘var’ with your Player objects to create chord progressions.

a = var([0,4,5,3], 4)
b1 >> bass(a, dur=PDur(3,8))
p1 >> pads(a + (0,2), dur=PDur(7,16))

You can add a ‘var’ to a Player object or a var.

b1 >> bass(a, dur=PDur(3,8)) + var([0,1],[3,1])
b = a + var([0,10],8)
print(int(Clock.now()), (a, b))

Updating the values of one ‘var’ will update it everywhere else

a.update([1,4], 8)
print(int(Clock.now()), (a, b))

Vars can be named …

var.chords = var([0,4,5,4],4)

And used later

b1 >> pluck(var.chords)

Any players using the named var will be updated

var.chords = var([0,1,5,3],4)

You can also use a ‘linvar’ that changes its values gradually over time. Change the value from 0 to 1 over 16 beats

c = linvar([0,1],16)

Run this multiple times to see the changes happening

print(int(Clock.now()), c)

Change the amp based off that linvar

p1 >> pads(a, amp=c)

a ‘Pvar’ is a ‘var’ that can store patterns (as opposed to say, integers)

d = Pvar([P[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], P[0, 1, 2, 3, 4, 5, 4, 3, 2, 1]], 8)
print(int(Clock.now()), d)
p1 >> pads(a, amp=c, dur=1/4) + d

Change the scale every 16 beats

Scale.default = Pvar([Scale.major, Scale.minor],16)

You even set the value to last forever once it is reached using a special value called “inf”

x = var([0, 1, 2, 3], [4, 4, 4, inf])
print(x) # Keep pressing - it will eventually stop at 3

Other types of TimeVar

There are several sub-classes of var that return values between the numbers specified. For example a linvar gradually change values in a linear fashion:

print(linvar([0,1],8)) # keep running to see the value change between 0 and 1

Increase the high-pass filter cutoff over 32 beats

p1 >> play("x-o-", hpf=linvar([0,4000],[32,0]))

Other types include sinvar and expvar

print("Linear:", linvar([0, 1], 8))
print("Sinusoidal:", sinvar([0, 1], 8))
print("Exponential:", expvar([0, 1], 8))

Offsetting the start time

Another useful trick is offsetting the start time for the var. By default it is when the Clock time is 0 but you can specify a different value using the “start” keyword

print(linvar([0, 1], 8))
print(linvar([0, 1], 8, start=2))

This can be combined with Clock.mod() to start a ramp at the start of the next 32 beat cycle:

d1 >> play("x-o-", hpf=linvar([0,4000],[32,inf], start=Clock.mod(32)))

It should be noted that when a Player() object uses a gradually changing TimeVar function, the value stored in it will be used at the time the note was triggered. This means that after playing a note you will not hear a change in value over time in the note itself. Try these lines of code for yourself:

No gradual change in high pass frequency:

p1 >> dirt(dur=4, hpf=linvar([0,4000], 4))

Apparent gradual change in high pass frequency:

p2 >> dirt(dur=0.25, hpf=linvar([0,4000], 4))

You can also use a duration of 0 to immediately skip the gradual change and move on to the next value. This is useful for “resetting” values and creating drops.

Raise the high pass frequency filter to 4000Hz, then back to 0:

p1 >> dirt(dur=0.25, hpf=expvar([0,4000], [8,0]))

As with normal TimeVars functions, TimeVars can be nested within other TimeVars as they gradually change to better manage the application of the values. For example, we can only increase the high pass filter frequency on the last 4 beats of a 32 beat cycle as follows.

Use a normal TimeVar function to set the value to 0 for 28 beats:

p1 >> dirt(dur=0.25, hpf=var([0,expvar([0,4000], [4,0])], [28,4]))

TimeVars as Patterns

Pvar(patterns,dur) >> So far we have only saved individual values in a TimeVar, but sometimes it makes sense to save an entire Pattern object.

You cannot do this with a regular TimeVar because each pattern in the input list of values is treated as a nested list of individual values. To avoid this behavior, you have to use a Pvar, short for Pattern-TimeVar (time variable pattern).

It is created just like any other TimeVar, but values can be entire lists/patterns.

a = Pvar([[0,1,2,3],[4,5,6]], 4)
print(Clock.now(), a)

Console output >> 0, P[0,1,2,3]

You can even nest a Pvar within a pattern like you would with a normal pattern to play alternate values.

Alternate the alternating notes every 8 beats:

p1 >> pluck([0,1,2,Pvar([[4,5,6,7],[11,9]], 8)], dur=0.25, sus=1)`


Scales

By default, Player Objects use the C Major scale. These can be changed by using the keyword arguments ‘scale’ and ‘root’. Scales can be defined as an array of semitones, such that the Major scale is [0,2,4,5,7,9,11] or one of the predefined scales from the Scale module, e.g. Scale.minor. Root refers to the tonic of the scale; 0 being C, 1 is C#, 2 is D and so on.

The default scale can be changed such that any Player not using a specific scale will be updated. This is done using the syntax below (each line is technically equivalent):

Scale.default.set("major")
Scale.default.set(Scale.major)
Scale.default.set([0,2,4,5,7,9,11])

Or the same thing, but minor:

Scale.default.set("minor")
Scale.default.set(Scale.minor)
Scale.default.set([0,2,3,5,7,10])

To save some time you can also do

Scale.default = "minor"

This is the same for the root:

Root.default.set(1)
Root.default.set("C#")

Or:

Root.default.set(2)
Root.default.set("D")

To see a list of all scales, use:

print(Scale.names())

You can change the scale used by a player using the ‘scale’ keyword

p1 >> pads([0,1,2], scale=Scale.minor)

Similarly, you can change the root note players using the root keyword and the Root.default object

p1 >> pads([0,1,2], scale=Scale.minor, root=2)


Groups

Groups are useful for controlling multiple player objects at the same time. A piano can consist of a bass line, chord line and melody line. Attributes such as volume can then be adjusted more easily. This is also useful if you want to arrange transitions with filter effects (e.g. high pass filters on the entire drum kit).

s1 >> piano(Pvar([[0,3,7,-2,0,5],[3,0,7,3,0]], [12,8]), oct=4, dur=PDur(3,8), sus=var([s1.dur,s1.dur*2], [6,2]), amplify=var([1,0.7], 8), amp=1)
s2 >> piano(Pvar([[2,5],[0,7]], 16), oct=var([5,6], [6,2]), dur=var([1,2], 32), amplify=var([0.8,1], 16), amp=1)
s3 >> piano((s1.degree,note), oct=(4,5), dur=var([PDur(3,8),1], PRand(8)), amplify=0.75, amp=1)
Piano.amp = Group(s1,s2,s3)

To turn the amplitude of this piano down, just use:

Piano.amp = 0

Or, set the volume on for 4 beats, then off for 4. This overrides existing amplitudes set in the player object:

Piano.amp=var([1,0],4)

To stop an entire group, use following command:

Piano.stop()

You can use functions to group things together. To execute use CTRL+Return, not ALT+Return.

def tune():
    b1 >> bass([0,3], dur=4)
    p1 >> pluck([0,4], dur=1/2)
    d1 >> play("x--x--x-")
tune()

or schedule the clock to call other grouped functions:

def verse():
    b1 >> bass([0,3], dur=4)
    p1 >> pluck([0,4], dur=1/2)
    d1 >> play("x--x--x-")
    Clock.future(16, chorus)
def chorus():
    b1 >> bass([0,4,5,3], dur=4)
    p1 >> pluck([0,4,7,9], dur=1/4)
    d1 >> play("x-o-")
    Clock.future(16, verse)
verse()

Several group objects already exist in Renardo for specific groups of player objects based on variable names ending with the suffix ‘_all’. So for every character, e.g. s there is a group called s_all, which contains s1,s2,s3,…,s9. So if you organize your players by variable names, you can easily apply effects or stop them all at once:

s1 >> pads([0,4,-2,3], dur=4)
s2 >> pluck([0,1,3,4], dur=0.25)

Use the group to apply the filter attribute to all player objects:

s_all.hpf = 500

This is also useful for:

s_all.amp = 0

With .stop() you can interrupt the entire group of players:

s_all.stop()

With .solo() all other player objects are muted, i. H. only the player objects of this group can be heard:

s_all.solo()

.only() stops all players who are not in the group:

s_all.only()


New utility player methods - fade and eclipse

If you would like to create more dynamic live loops in Renardo and subtle music a common need would be :

    to silence the loop periodically for example 4 beats every 32 beats to create automatic kinda breaks in the music
    to start a loop by fading in and stop by fading out to make the music smoother
    transition from a volume to another gradually

With FoxDot you could do that manually by using timevars and linear timevars with the amplify parameter like this:

b1 >> pluck([0,3,0,4,5], dur=.5, amplify=linvar([0,1], [16,inf], start=Clock.mod(4)))
# fadein will start at the beginning of next 4 beats bar and take 16 beats to rise
# then to fade out and stop
b1.amplify = linvar([1,0], [16,inf], start=Clock.mod(4))

b1.stop()

# to make a periodic pause of the loop (here 4 beat every 32 beat)
b1 >> pluck([0,3,0,4,5], dur=.5, amplify=var([1,0], [28,4]))

Renardo has new player methods to ease this while livecoding

b1 >> pluck([0,3,0,4,5], dur=.5).fadein(16)

b1 >> pluck([0,3,0,4,5], dur=.5).fadeout(16)

# fadeout will automatically stop the player when the volume reaches 0
# If you don't want it to stop you can use :

b1 >> pluck([0,3,0,4,5], dur=.5).fadeout(16, autostop=False)

# More generally if you want to gradually change the amplitude/volume of the loop
# from the current value to another you can use the fade method :

b1 >> pluck([0,3,0,4,5], dur=.5, amplify=0).fade(16, fvol=.5) # will fade from 0 to .5 on 16 beats

b1.fade(dur=4, fvol=.8) # will then fade from current .5 to .8 on 4 beats

b1.fade(dur=4, fvol=0, autostop=False) # will then fade to 0 while keeping the player running so you can fadein back later

Another cool point of fade, fadein and fadeout is that when the target value is reached the parameter is set back to a simple float value rather than keeping a linvar instance running until the end of time (inf).

For the periodic pause the utility method is called eclipse :

b1 >> pluck([0,3,0,4,5], dur=.5).eclipse(4, 32) # will pause the player 4 beats every 32 beats
# the silent period is at the start of the 32 beats cycle
# if you want to rather pause at the end or in the middle of the cycle :
b1 >> pluck([0,3,0,4,5], dur=.5).eclipse(dur=4, total=32, leftshift=16) # eclipse start at the 16th beat of the cycle
b1 >> pluck([0,3,0,4,5], dur=.5).eclipse(4, 32, 28) # eclipse start at the 16th beat of the cycle

There more details and similar methods so here is the code if you want and can read the signatures : https://github.com/e-lie/renardo/blob/68ef81a2755d6b608fdc2a18f80d8a1e31550834/renardo_lib/renardo_lib/Players.py#L2092

Otherwise all of this will be further documented soon.


Song arrangement
Pop/EDM Arrangement

A typical Pop Arrangement has Intro, Verses, Chorus, Bridge, Refrain, and Outro. There are different radiation of it, but that is the basics.

Common Structures for a song is as followed:

    Intro (4 Bars)

    1.Verse (8 -16 Bars) + Pre-Chorus (Optional)

    Chorus (8 - 16 Bars)

    2.Verse (8 - 16 Bars) + Pre-Chorus (Optional)

    Chorus (8-16 Bars)

    Outro (4 Bars)

Bars are 4 beats or beats. So 4 bars in Renardo means 16 counted as beats.

Further structures, whereas A is Verse, B is Chorus, C is Bridge:

ABABCA >> Verse / Chorus / Verse / Chorus / Bridge / Chorus AABA >> Verse / Verse / Bridge / Verse ABAB >> Verse / Chorus / Verse / Chorus (simplified version of the ABABCB) AAA >> Verse / Verse / Verse

The following is an example of a song structure in common electronic music:
							
Intro	Break	Buildup	Drop	Break	Buildup	Drop	Outro
16 Bars	16 Bars	4/8/16 Bars	16 Bars	16 Bars	4/8/16 Bars	16 Bars	16 Bars
Intro

    The Intro is pretty much anything you want it to be.
    Many songs start with just the melody that is rising up.
    You can even create a melodic question that is answered by the rest of the song or something of the sort.
    The important thing is to not stay too long at the Intro and make it tie in quickly.

Break/Bridge

    Less loud, less bass heavy, less instruments.
    This is used to break up what the listener has paid attention to. In electronic music, you usually take out the drums and add a rising sound to the next part.
    A Bridge/Break can be more powerful by adding new instruments or changing the key
    Try to keep this at 8 measures or less.
    The Bridge is a departure from what we’ve heard in a song, previously.
    This goes for both the lyrics and the music.
    Lyrically it’s an opportunity for a new perspective.
    Musically, it’s a chance to offer the listener something they haven’t heard before to keep the song interesting.

Buildup

    Goes usually from Break to Drop, can be even silence.
    It creates an emotional tension in the listener, which is then dissolved in the Drop.

Drop

    Loudest, most fun to listen to.
    The moment in a dance track when tension is released and the beat kicks in. This releases an enormous energy during a song’s progression.
    After the momentum Buildup, the pitch rising, the tension mounting, bigger, louder, until suddenly — the Drop.

Riser

    A Riser is just like a break except that it is arpeggiate or having some sort of buildup that is released with the next section coming in.
    Usually no beat and last 8 measures or 16.
    When the next part comes in, it will have a lot more energy and should be the climax of the piece.

Outro

    This is used to resolve the song and come in for a smooth landing.
    Some song’s don’t have an Outro and others have a long Outro.
    You can also add a final sense by adding a Coda, or strong cadence at the end of your track.


Chords
Chord

In music, a chord is the simultaneous sounding of at least three different tones that can be interpreted harmonically.

We can divide chords into different types depending on how many notes they have. We can have them in:

    Groups of two notes - called intervals or dyads
    Groups of three notes - known as triad chords
    Groups of four or more notes - usually called seventh chords or expanded chords

A triad occupies the root or first note of the scale, the third degree, and the fifth degree, with each interval being a third.

For example, the C minor scale has the notes C-D-Eb-F-G-Ab-B-C. Take the 1st, 3rd and 5th notes (C-Eb-G) to get a C to form a minor triad.

A seventh chord uses the root, 3rd, 5th, and 7th degree degrees, so a Cmin7 chord would add the Bb (C-Eb-G-Bb). C minor seventh chord.

Extended chords add the 9th, 11th, and 13th scales (the octaves of the 2nd, 4th, and 6th, respectively).
Chord Inversion

If you have a chord where the lowest note is not the note the chord is named after, we call this a chord inversion. A chord inversion takes a different starting note (also called a bass note) and builds the chord from there. Chord inversions are mainly used to allow easier voice guidance through different chord progressions, especially in the bass.

Features of a chord inversion are:

    The root note is not in the bases.

    Get a smooth dynamic by rearranging the chords through changing the octave of notes to align closer to first chord, thus in versing the highest note to the bass note.

    Inversion of a 5th becomes a 4th and visa verse.

    Major 2nd inverted becomes a minor 7th and a minor 7th becomes a Major 2nd.

    Major 6th inverted becomes a minor 3rd and a minor 3rd becomes a Major 6th.

Chord progression

To create a nice sounding and interesting melody, you need to carefully choose how each note moves to the next note and how each note relates to the notes in its vicinity. Notes can’t be too far apart, and usually you want the notes to stay within the key or related keys.

The same concept is used for harmony. Since a song usually consists of more than one chord, you need to relate each chord to the one before and after it in order for the harmonic movement to sound good and interesting. This is where a chord progression comes into play.

A chord progression is when several different chords are played one after the other.

Dur <<>> Moll

Major and minor form the two sides of the proverbial coin when it comes to defining the key of a song or composition. Songs are in either a major or a minor key. Sometimes more complex songs or pieces contain modulations (key changes), and we can see both major and minor keys represented in a single work. However, major and minor keys (and their correlating modes) cannot occur simultaneously, at least in tonal music.

Each piece or section of a piece must be either major or minor. You can’t be both. Major and minor songs are based on their respective scales (modes). This provides information about both the content of the melody and the harmony of a piece.

In other words, songs with a major key are selected from notes found in a particular seven-note major scale (like C major or F major, etc.). Songs tuned in minor are selected from seven-part minor scales (such as C minor or F minor, etc.). In the case of minor, however, there is a super-ordinate minor scale called the natural minor, as well as two variants, each called the harmonic and melodic minor.

In addition, major and minor chord progressions usually follow the primary cadences (harmonic touchstones) of the mode from which they are derived. Pieces tuned to major almost always end on a major home-base chord. This chord is usually referred to as I using Roman numerals.

The opposite is the case with songs in the minor key. Occasionally, however, classical pieces with a minor key surprise the listener by suddenly ending with a major third in the home-base or I chord. This unexpected switch gives the music a sudden boost. The classic term for this is Picardy third.

Create a minor from a major chord by lowering the 3rd, 6th and 7th degrees by one note.

Minor:
Moll	Dim	Dur	Moll	Moll	Dur	Dur

Major:
Dur	Moll	Moll	Dur	Dur	Moll	Dim

Am Example:
Am	B0	C	Dm	Em	F	G

Cm Example:
Cm	D0	D#	Fm	Gm	G#	A#

A minor scale can be achieved by lowering the 3rd, 6th and 7th major tones by one note

print(Scale.major)

Console output >> P[0,2,4,5,7,9,11]

print(Scale.minor)

Console output >> P[0,2,3,5,7,8,10]

If you only want to change one chord to minor, lower the third note.

A melodic minor scale is created by raising the 6th and 7th notes of the minor scale.

print(Scale.minor)

Console output >> P[0,2,3,5,7,8,10]

print(Scale.melodicMinor)

Console output >> P[0,2,3,5,7,9,11]

Examples 7th minor scale of E (E3,F#3,G3,A3,B3,C4,D4,E4)

    E3, G3, B3, D4 – m7 (add F#4 for m9)
    F#3, A3, C4, E4 – Dim7 (add G4 for Dim9)
    G3, B3, D4, F#4 – Maj7 (add A4 for Maj9)
    A3, C4, E4, G4 – m7 (add B4 for m9)
    B3, D4, F#4, A4 – m7 (add C5 for m9)
    C4, E4, C4, B4 – Maj7 (add D5 for Maj9)
    D4, F#4, A4, C5 – Major chord with minor7 – Dom7 (add E5 for Dom9)


Melody
Chords to Melody

One way is to create a chord progression, and than to find the melody in the chords.
E3	D3	F3	E3
C3	B2	D3	C3
A2	G2	A2	G2

As mentioned before, the octave in Renardo is 2 steps above the usual, middle C is 5 not 3.

Lets do the chords with 93 bpm with A as Root note and minor scale:

Clock.bpm = 93
Root.default=”A”
Scale.default=Scale.minor

# Chords:
chords = var([(0,2,4),(-1,1,3),(0,3,5),(-2,2,4)])
s1 >> swell(chords, oct=5, dur=4, sus=5)

# Hit the drums can help find a good melody:
b1 >> play("<X....X..X..[X.].X..><..o.><---->",sample=0)

The easiest way to start a melody is to take the highest notes of the chords. However, you want to add some non-chord notes to your chord notes:
E4	F4	D4	F4	D4	G4	E4	D4

seq=[4,5,3,5,3,6,4,3]
s2 >> pulse(seq, oct=6, dur=[3,1,3,3,1,1,2,2])

Melody to Chords

In this example we start with a melody in order to get suitable chords from it, here the melody.
A3	B3	C4	B3	E4	F4	C4	G4	E4	D4

Let’s set the tempo, the root and the scale:

Clock.bpm = 93

Root.default=”A”

Scale.default=Scale.minor

The originating Melody:

If you can’t remember the numbers on the scale list, use print(Scale.minor).

seq=[0,1,2,1,4,5,2,6,4,3]

Synth:

s1 >> saw(seq, dur=[2,1,1,4,3,1,1,1,1,1], formant=4, amplify=0.4)

The available chords (with 7th) for the notes played in the melody are as follows:
G4	A4	B4	C5	D5	E5	F5
E4	F4	G4	A4	B4	C5	D5
C4	D4	E4	F4	G4	A4	B4
A3	B3	C4	D4	E4	F4	G4

Here is a good example of a trip-hop-like track:
E4	D4	E4	E4
C4	B3	C4	G4
A3	G3	A3	C4

Let’s add the chords to the melody:

chords = var([(0,2,4),(-1,1,3),(0,2,4),(-1,6,4)])
s2 >> keys(chords, oct=4, dur=4, shape=0.4)

And a drum hit:

b1 >> play("<X....X..X..[X.].X..><..o.><---->", sample=0)

Add a counter melody (arpeggio)

Let’s keep it simple and use the chord notes to play with the chords. With the counter melody we want to add a rhythm to the track. As the 4th of the sequence in a 4-beat measure, we add the 2nd as shown here:

chords = var([(0,2,4),(-1,1,3),(0,2,4),(-1,4,6)])

becomes

seq2 = [0,2,4,2,-1,1,3,1,0,2,4,2,-1,4,6,4]

Now let’s add another instrument that plays the counter melody:

s3 >> karp(seq2, dur=1)

Chords to Bassline

In the example below, the chord progression is based on A minor, while raising a root to a higher octave and lowering a root.
G3	G3		
E3	F3	G3	G3
C3	D3	E3	E3
A2	B2	C3	B2

Let’s set the tempo, root and scale:

Clock.bpm = 128
Root.default=”A”
Scale.default=Scale.minor

Here chords and synth:

chords = var([(0,2,4,6),(1,1,3,6),(2,4,6),(1,4,6)])
s1 >> prophet(chords, oct=4, dur=4, sus=4)

The safe case is to use chord root notes as bass notes and lower those notes in the octave:
A1	G1	C2	E2

bassline1 = [0,-1,2,4]

Another way to create a bass line is to find notes within the chords (although the 7th can be tricky).
A1	B1	C2	B1

You can also change the duration of the bass line to get a rhythmic component:

Use dur=1, dur=[0.5,1], or dur=[1,2,1] instead of dur=4.

Another option is to move the root note of a chord one step up the previous chord row.

With dur=1:

bassline2=[0,0,0,-1,-1,-1,-1,2,2,2,2,-3,-3,-3,-3,0]

Or you use octave oct jumps:

bassline3=[0,0,0,0,-1,-1,-1,-1,2,2,2,2,-3,-3,-3,-3]

with dur=1 and oct=[3,3,4,3]

Finally, a melody as a bass line:
A1	G1	A1	B1	A1	G1	G1	A1	C2	C2	A1	G1	E1	E1	F1	G1

bassline4 = [0,-1,0,1,-1,-1,0,1,3,3,0,-1,-3,-3,-2,-1]

Bassline to Chords

We’ll start with tempo, root note, scale, and a simple bass line:

Clock.bpm = 128
Root.default=”A”
Scale.default=Scale.minor
bassline=[0,0,0,0,0,0,0,1]

Now let’s build chords along the minor chord, like: Am, Bm/A, G/A, Am.

chords = var([(0,2,4),(1,3,5),(0,2,4),(-1,1,3),(0,2,4)])

Bm/A and G/A mean “above A” because the bass line still keeps A as the root of the chord.

The corresponding synth examples for bass and chords are:

s1 >> jbass(bassline, oct=3, dur=0.5, shape=0.4) # Bass
s2 >> dirt(chords, oct=5, dur=[4,3,1,4,4], amplify=0.4) # Chords

Drums:

b1 >> play("<V....V..><..o.><....k..d>←--[--]>", sample=var([4, 2], 16), amplify=0.5)
b2 >> play(var(["[ss]",".[ss]"]), amplify=0.5)

Some additional notes on a bass line:

Only use one note at the time, as low frequency easy go “muddy”!


Scales and Modes
Scale

A musical scale, or scale, is technically defined as a series of ascending or descending unitary tones that form a range of notes that can be used to form a melody. Most of the scales in Western music correspond to a specific key. That is, a sequence of notes that is major or minor by default. This does not apply to the chromatic scale, which is a scale of all possible semitones in Western music. The whole tone scale is also a scale that consists of intervals that are two semitones apart.

Within a given key there are 7 notes in a single octave before reaching the 8th note, which has the same name as the first note and is twice the frequency. The seven notes have different intervals between adjacent notes. Sometimes it’s a semitone (semitone), sometimes it’s a whole tone (two semitones). The pattern of whole tone / semitone intervals that determine the notes of a key, starting with the note while the key is named, is whole-whole-half-whole-whole-whole-half. Within a single key, any of these seven notes could be used as the base note of an ascending sequence. Any such sequence created by starting with a different note in the key is a mode of that key, and each mode has a name. For example:

    Ionian - begins with the “tonic”; the note for which the clef is named. In the key of C, the Ionic mode begins with C. This mode is the most common and is colloquially referred to as the “major scale”. The pattern is WWHWWWH.

    Dorian - starts with the next note higher in key than the tonic (D, in the key of C). WHWWWHW.

    Phrygian - starts with the note that is a major third higher than the tonic (E). HWWWHWW.

    Lydian - begins with the note that is a full fourth higher than the tonic (F). WWWHWWH.

    Mixolydian - starts on the note that is a fifth higher than the tonic (G). WWHWWHW.

    Aeolian - begins with the note a major sixth higher than the tonic (A). This mode is also very important in modern music and is known as the “natural minor scale”. WHWWHWW.

    Locrian - begins with the note a major seventh higher than the tonic (Bb). HWWHWWW.

Scales table
Using Scale

    A scale is essentially a subset of the musical notes (pitches) between one note, e.g. C, and the same one an octave higher.

    The starting note is the key of the scale.

    Starting at C, these notes are:

    This set of all the notes is called the chromatic scale.

    If this was a Python list called chromatic, then chromatic[0] would return C, chromatic[1] would return C#, chromatic[2] would return D, and so until chromatic[11], which would return B.

    Because each musical scale is a subset of these pitches, we can think of each scale as a list of indices for accessing pitches in the chromatic scale.

chromatic = [C, C#, D, D#, E, F, F#, G, G#, A, A#, B]
C	C#	D	D#	E	F	F#	G	G#	A	A#	B(H)
0	1	2	3	4	5	6	7	8	9	10	11

    To see a list of the scales available just run command print(Scale.names()).

    By default, each player uses a globally accessibly default scale called Scale.default

    This can be changed in 3 ways:

Simply assigning the scale object to Scale.default:

Scale.default = Scale.minor

You can use the string name:

Scale.default = "minor"

You can also use the “set” method, which allows more options:

Scale.default.set("minor")

It is also possible to change the scale of players individually.

Force a player to use the minor scale:

p1 >> pluck([0,1,2,3], scale=Scale.minor)

Modes

W.I.P
Using Modes

W.I.P


Patterns
Pattern functions

PStep(n,value,default=0) >> Returns a pattern where every n-term is value, otherwise default.

s1 >> varsaw(PStep(3,[0,2,1,4,2,5],[-2,[-2,-1]]), oct=(4,6), dur=0.25, sus=0.125, lpf=linvar([200,4000], 8))

PSum(n,total,**kwargs) >> Returns a pattern of length n, the sum of which is total. For example: PSum(3,8) -> P[3,3,2] PSum(5,4) -> P[1,0.75,0.75,0.75,0.75].

s1 >> donk(P[:2], oct=[[5,6], 6], dur=PSum(12,8), sus=0.5)

PRange(start,stop=None,step=None) >> Returns a pattern equivalent to Pattern(range(start,stop,step)).

s1 >> piano([0,2,0,1], oct=4, dur=2, sus=1, amplify=0.7)
s2 >> piano(Pvar([[0,2,4,2],[0,4,2,1],PRange(0,8,var([2,1],4))], [4,4,8]), dur=Pvar([0.5,PDur([3,5],8)], [1,3]))

PTri(start,stop=None,step=None) >> Returns a pattern equivalent to Pattern(range(start,stop,step)) with the inverted shape appended.

s1 >> piano([0,2,0,1], oct=4, dur=2, sus=1, amplify=0.7)
s2 >> piano(Pvar([[0,2,4,2],[0,4,2,1],PTri(0,8,var([2,1], 4))], [4,4,8]), dur=Pvar([0.5,PDur([3,5], 8)],[1,3]))

PEuclid(n,k) >> Returns the Euclidean rhythm that distributes n pulses as evenly as possible over k steps. e.g. PEuclid(3,8) returns P[1,0,0,1,0,0,1,0].

s1 >> blip(Pvar([P[:2],P[:3]], 16), oct=4, dur=0.5, amplify=PEuclid([3,5,5,3],[7,8]))

PSine(n=16) >> Returns values of one cycle of a sine wave divided into n parts.

s1 >> fuzz(PSine(8), dur=0.5, sus=0.25, formant=1, room=0.5, mix=0.33, pan=PSine(32))

PDur(n,k,dur=0.25) >> Returns the actual duration based on Euclidean rhythms (see PEuclid), where dur is the length of each step. e.g. PDur(3,8) returns P[0.75,0.75,0.5].

s1 >> bass(PWalk(3), oct=5, dur=Pvar([PDur(5,7),PDur(5,8)], 16))
s2 >> pulse(Pvar([P[:3],P[:2]], 8), oct=5, dur=PDur(2,3), sus=0.125, lpf=expvar([400,4000], 16), lpr=0.75, amp=P10(16))

PBern(size=16,ratio=0.5) >> Returns a pattern of ones and zeros based on the ratio value (between 0 and 1). This is known as the Bernoulli sequence.

b1 >> play("S", sample=[1,3], amp=PBern(16,0.5))
b2 >> play("S", dur=PBern(24,0.5), delay=[0,0.5], sample=5, amp=1)

PBeat(string,start=0,dur=0.5) >> Returns a pattern of durations based on an input string, where non-spaces denote a pulse.

s1 >> donk(dur=PBeat(". . . ..", start=0, dur=[1]+[0.5]+[1]+[0.5]*2))
s2 >> bell(dur=PBeat(". . . ..", start=0, dur=0.5), amplify=0.6)

PSq(a=1,b=2,c=3)

s1 >> piano(PSq(1,2,3)-var([0,P[:2]*2], [4,8]))
print(PSq(1,2,3))

Pattern generators

PRand(lo,hi,seed=None)/PRand([values]) >> Returns a series of random integers between lo and hi, inclusive. If hi is omitted, the range is 0 to lo. A list of values can be provided in place of the range and PRand returns a series of values chosen at random from the list.

var.ch1 = var([PRand([0,2,4,8], seed=PxRand(200))], 4)
var.ch2 = var([PRand([0,1,3,5], seed=PxRand(200))], [8,4,4])
s1 >> piano([var.ch1,var.ch2], dur=0.5, amplify=0.6)

PxRand(lo,hi)/PxRand([values]) >> Identical to PRand, but no elements are repeated.

s1 >> pluck(PWalk(4), dur=PxRand([2,0.66,0.66,0.33,1,1,0.5,0.5,0.75]), oct=6, formant=3, tremolo=3, room=0.6, mix=0.3, amplify=0.65)

PwRand([values], [weights]) >> Uses a list of weights to indicate how often items with the same index are selected from the list of values. A weight of 2 means it is twice as likely to be picked as an item weighing 1.

s1 >> sitar(PWalk(4), dur=PwRand([2,0.66,0.33,1,0.5,0.75,0.25], [2,4,5,3,7,6,1]), oct=PwRand([6,6,7,5], [4,3,2,1]), room=0.6, mix=0.5, amplify=0.65)

P10(n)>> Returns a pattern of length n of a randomly generated series of ones and zeros.

s1 >> pulse(Pvar([[0,1],[0,2]], 16), oct=4, dur=2, sus=1, amplify=0.75)
s2 >> pulse(P[:4], dur=0.5, sus=0.25, amplify=0.75, amp=P10(16))

*PAlt(pat1, pat2, patN) >> Returns a pattern generated by alternating the values in the specified sequences.

0, -2, 0, 8, 2, 1, 0, 9, 4, 3, 7, 0, -2, 0, 5 …

mtf1 = [0,2,4]
mtf2 = [-2,1,3]
mtf3 = [0,0,2]
s1 >> piano(PAlt(mtf1,mtf2,mtf3,[8,9,7,5]), dur=0.5)

PJoin(patterns) >> Assembles a list of patterns.

mtf1 = [0,2,6,4]
mtf2 = [1,3,7,5]
s1 >> arpy(Pvar([mtf1,mtf2,mtf1,PJoin([mtf1,mtf2])], 8), oct=5, dur=0.5, formant=3, room=0.5, mix=0.3)

PPairs(seq,func=) >> Links a sequence to a second sequence obtained by executing a function on the original. By default, this is lambda n: 8-n.

s1 >> sitar(PPairs([0,4,2,0,6,4], lambda n: var([n*3,n-1], [12,4])), oct=4, dur=0.5, amplify=0.4)

PQuicken(dur=0.5,stepsize=3,steps=6) >> Returns a group of delay amounts that gradually decrease.

b1 >> play("m", dur=1, delay=[PQuicken(dur=2,stepsize=2,steps=3),PQuicken(dur=2,stepsize=2,steps=6)], sus=0.125, amplify=0.4)
b2 >> play("t", dur=4, delay=PQuicken(dur=1,stepsize=4,steps=3), sample=2, amplify=0.6)
b3 >> play("S", dur=4, delay=2+PQuicken(dur=0.5,stepsize=2,steps=3), amplify=0.65)

PRhythm(durations) >> Converts all tuples / PGroups into delays, which are calculated with the PDur algorithm.

b1 >> play("V", dur=PRhythm([0,0.5,0,0.25,1,0.75]), delay=0, sample=12, amplify=0.65)

PShuf(seq) >> Returns a mixed version of seq. This example uses a function to automatically shuffle the list.

def updateShuffle(n=0):
    beats=32
    if n % beats == 0:
         var.mtf = var([PShuf([0,1,3,4,-1])], 1)
    Clock.future(1, updateShuffle, args=(n+1,))
updateShuffle()
s1 >> ambi(var.mtf, oct=(5,6), dur=1, sus=0.25, echo=[0,0.5], echotime=2, room=0.66, mix=0.3, amplify=0.5)

PStretch(seq,size) >> Returns seq as a pattern and is looped until its length is size, e.g. PStretch ([0,1,2], 5) returns P [0,1,2,0,1].

var.mtf1 = var([0,1,2,4,[3,5],0,2,4], 0.5)
s1 >> karp(PStretch(var.mtf1,12), oct=6, dur=[0.5,0.66], shape=0.125, formant=0, rate=0.125, amplify=0.66)

PStrum(n=4)

var.mtf1 = var([0,1,2,0,[4,2],3,-2,[-1,4]], 0.5)
s1 >> marimba(var.mtf1, oct=var([5,6], [0.5,1.5]), dur=Pvar([PStrum(5),PStrum(2)], 16), shape=0.25, room=0.5, mix=0.5, amplify=1)

PStutter(seq,n=2) >> Creates a pattern so that each element in the array is repeated n times (n can be a pattern).

var.mtf1 = var([0,6,4,2], 2)
s1 >> quin(PStutter([var.mtf1], 2), oct=4, dur=PStutter([1,0.5], 4), sus=0.25, amplify=0.65)

PZip(pat1, pat2, patN) >> Generates a pattern that ‘zips’ multiple patterns. PZip([0,1,2], [3,4]) creates the pattern P[(0,3),(1,4),(2,3),(0,4),(1,3),(2,4)].

s1 >> faim(PZip([0,2], [2,-2,4,6]), oct=6, dur=2, atk=0.15, chop=2, lpf=1800, vib=2, amplify=0.5)

PZip2(pat1,pat2,rule=) >> Like PZip, but only uses two patterns. Connects values if they meet the rule.

s1 >> faim(PZip2([0,2], [2,-2,4,6], rule=<lambda>), oct=6, dur=2, atk=0.15, chop=2, lpf=1800, vib=2, amplify=0.5)

Pvar >> TimeVar, which saves lists instead of individual values (var,sinvar,linvar,expvar).

s1 >> gong(P[Pvar([[0,2],[2,4],[4,6],[2,4]], 2)], dur=0.5, lpf=expvar([800,8000], [4,0]), pan=sinvar([-0.65,0.65], 8), amplify=0.75)

PWhite(lo,hi) >> Returns random floating point numbers between lo and hi.

s1 >> arpy((0, var(PRand([Scale.default]), 8)), oct=var([5,6], [24,8]), dur=PDur(5,8), room=0.5, mix=sinvar(0.3,0.75), pan=PWhite(-1,1), amplify=0.65)

PChain(mapping_dictionary) >> Based on a simple Markov chain with equal probabilities. Takes a dictionary of elements, states, and possible future states. Every future state has an equal chance of being selected. If a possible future state is not valid, a KeyError is raised.

s1 >> rave(PChain([0,8,6,3,-2,0,-3]), dur=0.25, sus=0.125, amplify=0.5)

PWalk(max=7,step=1,start=0) >> Returns a series of integers with each element an increment apart and with a value in the range of +/- the maximum. The first element can be selected with start.

s1 >> dirt(PWalk(6,2), dur=[0.5,PSum(4,3)], oct=6, shape=0.3, lpf=1800, pan=(-0.65,0.65), amplify=0.25)

PFibMod() >> Returns the Fibonacci sequence.

s1 >> feel(PFibMod()[:7]+var([0,-3,0], 8), dur=1, shape=0.25, chop=128, room=0.75, mix=0.5)


Chord Progressions
Example “Billy Jean”

This example will show how to code “Billy Jean”s’ Intro by Michael Jackson.

    Scale: minor

    Root: E

    Chords:

The number after it refers to the octave. In Renardo, the middle C=5, so you always have to add 2 when composing from the sheet music.

# Tempo:
Clock.bpm=117
# Root E:
Root.default=“E”
# Scale to minor:
Scale.default=Scale.minor
# Chords in a list:
chords=[(0,2,4),(0,1,3,5),(0,2,4,6),(0,1,3,5)]
# Player object:
s1 >> pluck(chords, oct=3, dur=[1.5,5/2], sus=2)
# Drums:
b1 >> play("<V....V..V...[VV]V..><..o.><---->")

Example “Get Lucky”

This example will show how to create bassline and chords of the track “Get Lucky” by Daft Punk.

Bass:
B1	D2	F#2	E2

Chords:
Bm	D	F#m	Em

In the fourth chord there is a note borrowed from the neighbor F#m (Circle of Fifths):
F#2	A2	C#3	B2
D2	F#2	A2	G#2 (F#m chord key)
B1	D2	F#2	E2

As an extra, you can try to create a little variety using TimeVars:

Drop: Thinner no beats Break: No voice Buildup: Mix BreakNDrop

With 4 notes/chords played every 16 beats, the song structure is as follows:
Intro	Break	Buildup	Drop	Break	Buildup	Drop	Outro
16 Beats	32 Beats	32 Beats	64 Beats	32 Beats	32 Beats	64 Beats	48 Beats


Synth Attributes
A Piano

In the example below, 3 players are used to create a full piano:

p1 >> piano([0,1,0,-1], oct=4, dur=2, amplify=0.75)
p2 >> piano([(2,4),(0,2),(3,5),(1,3),(2,4),(0,2),(-1,1),(-3,-1)], dur=1, amplify=0.66)
p3 >> piano([0,4,2,4,1,2,1,3,2,3,5,7,-1,3,-3,1], oct=6, dur=0.5).every(32, "reverse")

This is the same as:

bassline = [0,1,0,-1]
chords = [(2,4),(0,2),(3,5),(1,3),(2,4),(0,2),(-1,1),(-3,-1)]
melody = [0,4,2,4,1,2,1,3,2,3,5,7,-1,3,-3,1]
p1 >> piano(bassline, oct=4, dur=2, amplify=0.75)
p2 >> piano(chords, dur=1, amplify=0.66)
p3 >> piano(melody, oct=6, dur=0.5).every(32, "reverse")

Another example

Scale.default="minor"
Root.default.set(var([1, 2], 32))
Clock.bpm=105
a1a = P[2, 6, 4, -2]
a1b = P[var([0, 2, -2, 2], 16), 4, 8]
a1c = P[var([[0, P*(0, 0, 0, 0, var([0, 8, 6, 4], 16))], 0], 16)]
a1d = P[var([[P*(8, 7, 6, 5, 4), P*(4, 6, 8)], 0], 16)]
a1 >> pianovel(
    (a1a, a1b, a1c, a1d),
    amp=(0.4 * var([linvar([1, 0.2], 0.25), 1, PBern(16, 0.9)], 16), var([0.4, 0.6], 4)), dur=(1,2),
    oct=(3,6),
    vib=0.5, vibdepth=0.5,
    lpf=(var([0, 600], 32),linvar([400, 4000], 64)),
    chop=(linvar([0, 4], 64), 0),
    shape=(0.2, 0), formant=(0, var([1, 0], 4)),
    slide=var([0, var([2, -0.5, 0], 3)], [3, 1]),
    pan=(expvar([0, -0.5], 12), expvar([0, 0.5], 16))
)


Sample Attributes
Attribute pshift

Here the ones, that work with samples are following:

dur, delay, sample, sus, pan, slide, slidedelay, glide, glidedelay, bend, benddelay, coarse, striate, rate, pshift, hpf, hpr, lpf, lpr, swell, shape, chop, tremolo, echo, echotime, spin, cut, verb, room, mix, formant, shape, drive, blur

For example, if you use pshift, you can change the pitch of the sample:

b1 >> play("#", dur=2, pshift=linvar([0,8], 16))


Scales
Play through all scales

Use the following code to iterate through all of the available scales Renardo provides.

Displays all available scales:

print(Scale.names())

Assign the selected scale as the default:

Scale.default=Scale.chromatic

Variable to assign a step to each note on the scale:

steps=len(Scale.default)

Play the notes through the scale:

p1 >> pluck(P[:steps])


Beats
Create beats

    Try to add variations, modulations, and/or swing (e.g. attribute nugde) to your beats to keep it alive.
    Variations are changes in the beat structure from one bar to another.
    Modulations are effects on the entire drum set, or on single parts of the drum set.
    Adjust some off notes to get a different dynamic within the beat, give your beat some swing.
    Be careful not to get to dynamic, thus losing the drive through the bass drum.

Start with the basic pattern made of a kick, a snare, and a HiHat.

k1 >> play(“X...X...”)
s1 >> play(“..o...o.”)
h1 >> play(“-.-.-.-.”)

. (dot) is used as a placeholder to make it easier to see.

As we will increase our beat in the future, leave 3 player for the drum or drum-like sounds, 3 player for snare and snare-like sounds, and 3 for HiHat, OpenHat s.o.

Now lets add a variation to the HiHat:

h1 >> play(“-.-.-.-.”).every(16,”mirror”).every(8,”stutter”,2)

And here another example:

h2 >> play(“--------”, sample=3, amplify=[0.3,0.3,0.6,0.3,0.3,0.6,0.3,0.6])

You also can add ghost notes, that are usually quieter 16 offbeat notes before or after the main note. For this, we will use <> for layering to adjust the volume to the ghost note:

ks >> play(“<..o...o.><.[.o]......>”, amplify=(0.7, 0.3))

The following examples will help you to experience the concept using familiar rhythms and beats. In addition, use your own arguments.
House

Tempo:
Clock.bpm=128

BassKick:
b1 >> play("X.", rate=0.8, sample=2, amplify=0.6)

Clap:
b4 >> play("..*.", sample=3, amplify=0.4)

Snare:
b5 >> play("......o.", rate=1.4, sample=1, amplify=0.5)

HiHat:
b7 >> play(".-", rate=0.8, sample=3, delay=PRand([0,Pwhite(-0.5,0.5)]), amp=0.6)

Cymbal:
b8 >> play("#", rate=1.2, dur=16, sus=8, amplify=0.8)

Drum N Bass

Tempo:
Clock.bpm=170

BassKick:
b1 >> play("V....V..VV...V..", rate=0.8, sample=2, amplify=0.6)
b2 >> play("v......[vvvv]", sample=4, amplify=0.6)

Snare:
b4 >> play(Pvar(["..o.","..o[.o.]"], [12,2]), sample=2, amplify=0.4)
b5 >> play("..i.", amplify=PRand([0.4,PWhite(0.6,0.4)]))

Shaker:
b7 >> play("s", rate=0.8, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)]))

Closed HiHat:
b8 >> play("-", rate=1.4, pshift=linvar([0,16], 8), sample=2, shape=0.3, amplify=1.2)

Dubstep

Tempo:
Clock.bpm=140

BassKick:
b1 >> play(Pvar(["V...V...", "V[..V.]..[V..V][..V.].[..V.]"], 16), dur=1, rate=1.2, sample=6, amplify=0.6)
b2 >> play(Pvar(["X...X...", "X[..X.]..[X..X][..X.].[..X.]"], 16), dur=1, sample=2, amplify=0.6)
b3 >> play(Pvar(["v...v...", "v[..v.]..[v..v][..v.].[..v.]"], 16), dur=1, sample=4, amplify=0.6)

Snare:
b4 >> play(Pvar(["..o...o.", "..o...oo"], 16), dur=1, rate=0.75, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)]))
b5 >> play(Pvar(["..i...i.", "..i...ii"], 16), dur=1, sample=4, amplify=0.4)
b6 >> play(Pvar(["..h...h.", "..h...hh"], 16), dur=1, sample=5, amplify=0.4)

Closed HiHat:
b7 >> play("-", dur=0.5, rate=0.4, pshift=linvar([0,8], 8), sample=4, amplify=0.8)
b8 >> play("s", dur=0.5, rate=1, sample=1, amplify=PRand([0.4,PWhite(0.6,0.4)]))

BuildUp:
c1 >> play("V.", dur=Pvar([1,0.5,0.25,0.1], [16,8,4,4]), rate=1.2, sample=6, amplify=Pvar([0.6,0], [30,2]))
c2 >> play("X.", dur=Pvar([1,0.5,0.25,0.1], [16,8,4,4]), sample=2, amplify=Pvar([0.6,0], [30,2]))
c3 >> play("v.", dur=Pvar([1,0.5,0.25,0.1], [16,8,4,4]), sample=4, amplify=Pvar([0.6,0], [30,2]))
c4 >> play("o.", dur=Pvar([1,0.5,0.25,0.1], [16,8,4,4]), rate=0.75, sample=2, amplify=Pvar([0.4,0], [30,2]))
c5 >> play("i.", dur=Pvar([1,0.5,0.25,0.1], [16,8,4,4]), sample=4, amplify=Pvar([0.4,0], [30,2]))
c6 >> play("h.", dur=Pvar([1,0.5,0.25,0.1], [16,8,4,4]), sample=5, amplify=Pvar([0.4,0], [30,2]))

Create Groups to control a bunch of Player() objects at the same time:
gB = Group(b1,b2,b3,b4,b5,b6,b7,b8)
gC = Group(c1,c2,c3,c4,c5,c6)

Use a TimeVar to swap between Build and Drop:
gB.amp=var([1,0], [64,32])
gC.amp=var([0,1], [64,32])


_Add stretch, pshift, rate or reverse to create different patterns!_
``` Drum N Bass


```python
Tempo:
Clock.bpm=170

BassKick:
b1 >> play("V....V..VV...V..", rate=0.8, sample=2, amplify=0.6)
b2 >> play("v......[vvvv]", sample=4, amplify=0.6)

Snare:
b4 >> play(Pvar(["..o.","..o[.o.]"], [12,2]), sample=2, amplify=0.4)
b5 >> play("..i.", amplify=PRand([0.4,PWhite(0.6,0.4)]))

Shaker:
b7 >> play("s", rate=0.8, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)]))

Closed HiHat:
b8 >> play("-", rate=1.4, pshift=linvar([0,16], 8), sample=2, shape=0.3, amplify=1.2)

Trap

Tempo:
Clock.bpm=140

BassKick:
b1 >> play("[VV]..V[.V]V.[.V].V..V.V.V.", dur=1, rate=1.2, sample=-1, amplify=0.6)
b2 >> play("[XX]..X[.X]X.[.X].X..X.X.X.", dur=1, sample=2, amplify=0.6)

Snare:
b4 >> play("..o.", dur=1, rate=0.75, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)]))
b5 >> play(".H..", dur=1, rate=1.4, sample=1, pan=(-0.7,0.7), amplify=0.4)

Closed HiHat:
b7 >> play("[--]", dur=PRand([4,2,1,0.5,PDur(3,8)*2,PDur(3,7)*2], 0.25), rate=0.75, sample=3, amplify=0.4)
b8 >> play("[--]", dur=PRand([4,2,1,0.5,0.25]), rate=0.5, sample=-1, amplify=0.4)


Here are a few instruments:
s1 >> dub(PRand([0,2,3], 0.25), oct=(3,4), dur=4, chop=PRand([6,8]), shape=0.6, amplify=0.3)
s2 >> space(s1.degree, oct=(4,5), dur=4, chop=PRand([3,4]), room=0.4, mix=0.5, amplify=1.2).offbeat()
s3 >> pulse([2,3,5,7,9], oct=var([3,4,5]), dur=PRand([0.5,0.25], 6), shape=0.6, formant=var([3,0,2], 0.5), room=0.75, mix=0.5, pan=[-0.6, 0.6], amplify=0.4)

HipHop

Tempo:
Clock.bpm=80

BassKick:
b1 >> play("X..X....X.XX....", rate=var([0.8,1], 8), formant=2, sample=5, amplify=1.3, amp=1)

Snare:
b4 >> play("..i.", rate=0.75, sample=2, amplify=PRand([0.4, PWhite(0.6,0.4)]))
b5 >> play(".H.......H......", dur=0.5, rate=1.4, sample=1, delay=1/16, pan=(-0.7,0.7), amplify=0.4)

Closed HiHat:
b7 >> play("--.-", rate=0.75, sample=3, amplify=0.4)

Open Hat / Shaker:
b8 >> play(".............#..", rate=1.4, sample=2, amplify=1, amp=1)
b9 >> play("[ss]", rate=0.75, sample=2, hpf=linvar([800,6000], 1), amplify=0.4, amp=1).every(PRand([4,8,12,16]),"stutter",PRand([2,3,4]))

Footwork

Tempo:
Clock.bpm=154

BassKick:
b1 >> play("X..X..X.X..X..X.", dur=1, rate=6/5, sample=-1, amplify=0.6)
b2 >> play("V..V..V.V..V..V.", dur=1, sample=1, amplify=0.6)
b3 >> play("{([XX])([X.])([X...])}", dur=1, rate=PRand([0.75,0.4,1,1.4], 0.25), shape=linvar([1.2, 0.4], 16), amplify=0.4, amp=1).every(PRand([2,4,8,16]),"stutter",PRand([2,3,5]))

Snare:
b4 >> play("............H...", dur=1, rate=0.75, sample=2, amplify=PRand([0.4,PWhite(0.6,0.4)])).every(PRand([4,8,12]),"stutter", PRand([2,3]))
b5 >> play("......o.......o[oo.o]",dur=1, rate=7/5, sample=1, pan=(-5/7,5/7), amplify=0.4)
b6 >> play("i",dur=PRand([4,2,1,0.5, PDur(3,8)*2,PDur(3,7)*2], 0.25), rate=0.5, sample=3, amplify=0.4)

HiHat:
b7 >> play("..-.....", rate=0.75, sample=3 , amplify=0.4)
b8 >> play("-", dur=1, sample=3, amplify=0.8)
b9 >> play("{([--])(M)}", dur=1, rate=PRand([0.75,0.4,1,1.4], 0.25), sample=2, shape=linvar([1/7,0.4], 16), amplify=1/5, amp=1).every(PRand([2,4,8,16]),"stutter", PRand([3,5]))

Funk

Tempo:
Clock.bpm=118

BassKick:
b1 >> play("VV...[VV]..", dur=0.5, rate=1.2, sample=-1, amplify=0.4)
b2 >> play("VV...[VV]..", dur=0.5, sample=linvar([0,5], 4), amplify=0.4)

Snare:
b4 >> play("..[o.][.o][.o].[o.][.o]", dur=0.5, rate=2, sample=5, amplify=PRand([0.4,PWhite(0.3,0.4)]))
b5 >> play("....i..i.i..i..i", dur=0.25, rate=1, sample=3, pan=(-0.7, 0.7), amplify=0.4)
b6 >> play("..o.", dur=1, rate=2, sample=5, pan=(-0.7, 0.7), amplify=0.4)

HiHat:
b7 >> play("[-.-.][-.--][-...][-.-.][--..][-.-.][-.-.][-...]", dur=1, rate=1, sample=2, amplify=1)
b8 >> play("[-.]", dur=0.5, rate=1, sample=-1, amplify=0.8, amp=1)
b9 >> play("[ll].-.", sample=var([3,4,0], 16), formant=linvar(5,8), amplify=0.8, amp=1).every(PRand([2,4,8,16]),"stutter", PRand([2,3,5]))
b0 >> play("[ss]", rate=1, sample=2, shape=0.6, amplify=0.8, amp=1).every(PRand([2,4,8,16]),"stutter", PRand([2,3,5]))

gBeats = Group(b1,b2,b3,b4,b5,b6,b7,b8,b9,b0)
gBeats.amp=1


Transitions
Create transitions

    Ramp up, then breath, then beat again (drum roll…silence…beat). Here one can excellently the group assignments z. B. with gBeats.hpf = linvar([0,5000], [12,0], start = Clock.now()), then all of a sudden gBeats.amp = var([0,1], [4 ,inf], start=Clock.now())
    To start a transition with the next bar just use start=nextbar instead.
    Subtract before you add, like no bass beat only snare and HiHat.
    Roll and ramp it up with 8th and 16th notes of e.g. snare, HiHat, Shaker.
    If you need a transition from one section to another, without big subtraction like taking drum beat out, be subtle.


How to code SynthDefs

Renardo creates music by giving player objects a digital instrument to play, which are called SynthDefs. You can see the list of pre-installed ‘Synths’ by executing

print(SynthDefs)

Each one of these represents a SynthDef object. These objects are then given to Players to play - like giving an instrument to someone in your orchestra.
Writing your own Synth Definitions

This is a bit more advanced, but if you have already written SynthDefs in Supercollider then you might feel at home. If not, the SuperCollider Book will help you getting started with SuperCollider.

Renardo can access any SynthDef stored on the SuperCollider server, but it needs to know it’s there. If you have already written a SynthDef in SuperCollider and named it \mySynth then you just create a SynthDef instance using Renardo like so:

mySynth = SynthDef("mySynth")

Using the same variable name in Renardo as in SuperCollider for your SynthDef is a good idea to avoid confusion. If you want to write (or edit) your own SynthDef during run-time in Renardo you can use a SuperCollider API by importing the SCLang module. All Renardo SynthDef objects inherit the base-class behaviour, such as low- and high-pass filters and vibrato, but these can be overridden or updated easily. If you want to know more about digital sound processing and SynthDef creation, check out the SuperCollider Documentation. Below is an example of creating one in Renardo:

Import module for writing SCLang code from Python

from SCLang import *

Create a SynthDef named ‘example’ (using the same variable name as the SynthDef name is a good idea)

example = SynthDef("example")

Create the oscillator (osc) using a sine wave

example.osc = SinOsc.ar(ex.freq)

And give it a percussive sound envelope (env)

example.env = Env.perc()

Finally, store it!

example.add()

How to create a SynthDef

with SynthDef("pads") as pads:
  pads.osc = SinOsc.ar(pads.freq)
  pads.env = Env.perc()

Equivalent to:

pads = SynthDef("pads")
pads.osc = SinOsc.ar(pads.freq)
pads.env = Env.perc()
pads.add()

