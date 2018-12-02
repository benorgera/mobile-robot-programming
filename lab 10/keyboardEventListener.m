function keyboardEventListener(~,event)
%keyboardEventListener Invoked when a keyboard character ispressed.
global keypressFrame;
global keypressDataReady;
global keypressKey;
keypressFrame = keypressFrame + 1;
keypressDataReady = 1;
keypressKey = event.Key;
end