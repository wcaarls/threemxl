/*
 * OptionVars.h
 *
 *  Created on: Apr 21, 2010
 *      Author: Erik Schuitema
 *
 *  A COptionVar keeps track of whether it has been assigned or not.
 *
 *  Example:
           configSection.get("steptime", &mOptionDbl);
           logNoticeLn(CLog2("temp"), "After reading steptime, isSet() = " << mOptionDbl.isSet() << ", value = " << mOptionDbl);
           mOptionDbl.reset();
           configSection.get("nonexistent", &mOptionDbl);
           logNoticeLn(CLog2("temp"), "After reading rubbish, isSet() = " << mOptionDbl.isSet() << ", value = " << mOptionDbl);

	Output:
           [temp] NTC: After reading steptime, isSet() = 1, value = 0.0133333
           [temp] NTC: After reading rubbish, isSet() = 0, value = 0.0133333
 *
 */

#ifndef OPTIONVARS_H_
#define OPTIONVARS_H_

template<class T>
class COptionVar
{
	protected:
		bool	mSet;
		T		mValue;

	public:
		COptionVar()									{mValue = 0; mSet=false;}

		bool isSet()									{return mSet;}
		void reset()									{mSet=false;}
		// The object can be cast to a const reference of its native type T
		operator const T&()								{return mValue;}
		// Assignment of a value of its native type T
		COptionVar<T>& operator =(const T& newValue)	{mValue = newValue; mSet = true; return *this;}
};

typedef COptionVar<bool>			COptionBool;
typedef COptionVar<int>				COptionInt;
typedef COptionVar<double>			COptionDouble;
typedef COptionVar<char>			COptionChar;
typedef COptionVar<unsigned char>	COptionByte;
typedef COptionVar<unsigned short>	COptionWord;

#endif /* OPTIONVARS_H_ */
