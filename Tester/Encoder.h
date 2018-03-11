/*
  Encoder.h - Library for a simple encoder with one button.
  Created by Przemysław T. Piotrowiak, March 28, 2017.
  Released into the public domain.
*/

#ifndef Encoder_h
#include "Arduino.h"
class Encoder
{
	public: Encoder(byte pinA, byte pinB, byte pinButton, int range);
	bool IsButtonPressed();
	void AckPosChange();
	bool AskPosChange();	
	int GetPosition();
  bool HasRotated();	

	private:
	
	byte read_gray_code_from_encoder(); //zwraca stan enkodera jako jeden z 4 stanow
	void encoder_rotation(byte prev, byte current);
	int _range; // ilosc "pozycji" enkodera
	int _pinA; // pin A enkodera
	byte _pinB; // pin B enkodera
	byte _pinButton; // pin przycisku enkodera

	// These properties maintain current state
	byte encoder_state; //przechowuje stan wejsc pinA i pinB
	byte encoder_state_temp;// = read_gray_code_from_encoder(dtState, clkState);
	int currentPosition; //aktualna "pozycja" enkodera
	bool button; // stan przycisku
	bool rotated; // wskaznik zmiany pozycji
};

#endif
