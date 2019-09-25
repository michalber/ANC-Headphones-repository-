/*
  ==============================================================================

    FilterVisualizer.h
    Created: 24 Sep 2019 3:09:27pm
    Author:  michu

  ==============================================================================
*/

#pragma once

#include "../examples/Assets/DemoUtilities.h"

class  FilterVisualizer :  public Component,
							private Timer
{
    struct Settings {
        float fMin = 20.0f;    // minimum displayed frequency
        float fMax = 20000.0f; // maximum displayed frequency
        float dbMin = -50.0f;  // min displayed dB
        float dbMax = 30.0f;   // max displayed dB
        float gridDiv = 5.0f;  // how many dB per divisions (between grid lines)
        bool gainHandleLin = false; // are the filter gain sliders linear?
    };
    
    struct FilterWithSlidersAndColour {
        dsp::FIR::Coefficients<float>::Ptr* coefficients;
        Colour colour;
        float* overrideGain = nullptr;
    };
    

    const float mL = 23.0f;
    const float mR = 10.0f;
    const float mT = 7.0f;
    const float mB = 15.0f;
    const float OH = 3.0f;
    
public:
    FilterVisualizer() : Component(), overallGainInDb(0.0f), sampleRate(48000) 
	{
		startTimerHz(2);
	}
    FilterVisualizer(float fMin, float fMax, float dbMin, float dbMax, float gridDiv, bool gainHandleLin = false) : Component(), overallGainInDb(0.0f), sampleRate(48000), s{fMin, fMax, dbMin, dbMax, gridDiv, gainHandleLin} 
	{
		startTimerHz(2);	
	}
    ~FilterVisualizer() {};
    
	void updateCoefficients(dsp::FIR::Coefficients<float>::Ptr newCoeffs) {
		FilterWithSlidersAndColour& handle(elements.getReference(0));
		dsp::FIR::Coefficients<float>::Ptr coeffs = *handle.coefficients;
		coeffs = newCoeffs;
	}

	void timerCallback() override
	{
		repaint();
	}

    void paint (Graphics& g) override
    {
        //g.setColour(Colours::steelblue.withMultipliedAlpha(0.01f));
		g.setColour(Colour(39, 50, 56));
        g.fillAll();
        
        //int width = getWidth();
        
        g.setFont(getLookAndFeel().getTypefaceForFont (Font(12.0f, 2)));
        g.setFont(12.0f);
        
        // db labels
        float dyn = s.dbMax - s.dbMin;
        int numgridlines = dyn/s.gridDiv+1;
        
        //g.setFont(Font(getLookAndFeel().getTypefaceForFont (Font(10.0f, 1)));
        //g.setColour (Colours::white);
		g.setColour(Colour(137, 176, 196));
        for (int i=0; i < numgridlines; i++)
        {
            float db_val = s.dbMax - i*s.gridDiv;

            int ypos = dbToY(db_val);
            
            String axislabel = String((int)db_val);
            g.drawText (axislabel, 0, ypos-6, 18, 12.0f, Justification::right, false);
        }


        // frequncy labels
        for (float f=s.fMin; f <= s.fMax; f += powf(10, floorf(log10(f)))) {
            int xpos = hzToX(f);
            
            String axislabel;
            bool drawText = false;
            
            if ((f == 20) || (f == 50) || (f == 100) || (f == 500))
            {
                axislabel = String((int)f);
                drawText = true;
            }
            else if ((f == 1000) || (f == 5000) || (f == 10000) || (f == 20000))
            {
                axislabel = String((int)f/1000);
                axislabel << "k";
                drawText = true;
            }
            
            if (drawText)
            {
                g.drawText (axislabel, xpos - 10, dbToY(s.dbMin) + OH + 0.0f, 20, 12, Justification::centred, false);
            }
        }


        //g.setColour (Colours::steelblue.withMultipliedAlpha(0.8f));
		g.setColour(Colour(204, 238, 255).withMultipliedAlpha(0.3f));
        g.strokePath (dbGridPath, PathStrokeType (0.5f));

        //g.setColour(Colours::steelblue.withMultipliedAlpha(0.9f));
		g.setColour(Colour(204, 238, 255).withMultipliedAlpha(0.2f));
        g.strokePath (hzGridPathBold, PathStrokeType (1.0f));

        //g.setColour(Colours::steelblue.withMultipliedAlpha(0.8f));
		g.setColour(Colour(204, 238, 255).withMultipliedAlpha(0.3f));
        g.strokePath (hzGridPath, PathStrokeType (0.5f));
               
        
        // draw filter magnitude responses
        Path magnitude;
        allMagnitudesInDb.fill(overallGainInDb);
        
        int xMin = hzToX(s.fMin);
        int xMax = hzToX(s.fMax);
        int yMax = dbToY(s.dbMin);
        int yMin = dbToY(s.dbMax);
        int yZero = filtersAreParallel ? yMax + 10 : dbToY(0.0f);
        
        g.excludeClipRegion(Rectangle<int>(0.0f, yMax+OH, getWidth(), getHeight()-yMax-OH));
        
        if (filtersAreParallel)
            complexMagnitudes.fill(std::complex<double>());
        
        
        for (int b = elements.size(); --b >= 0;) {
            bool isActive = activeElem == b;
            magnitude.clear();
            
            FilterWithSlidersAndColour& handle(elements.getReference(b));
            dsp::FIR::Coefficients<float>::Ptr coeffs = *handle.coefficients;
            //calculate magnitude response
            if (coeffs != nullptr)
                coeffs->getMagnitudeForFrequencyArray(frequencies.getRawDataPointer(), magnitudes.getRawDataPointer(), numPixels, sampleRate);
            float additiveDB = 0.0f;
            //if (filtersAreParallel && handle.gainSlider != nullptr)
            //    additiveDB = handle.gainSlider->getValue();
                    //FloatVectorOperations::multiply(magnitudes.getRawDataPointer(), Decibels::decibelsToGain(handle.gainSlider->getValue()), numPixels);
            
            //calculate phase response if needed
            if (filtersAreParallel && coeffs != nullptr)
                coeffs->getPhaseForFrequencyArray(frequencies.getRawDataPointer(), phases.getRawDataPointer(), numPixels, sampleRate);
            
            float db = Decibels::gainToDecibels(magnitudes[0]);
            magnitude.startNewSubPath(xMin, jlimit((float) yMin, (float) yMax + OH + 1, dbToYFloat(db)));
            
            for (int i = 1; i < numPixels; ++i)
            {
                float db = Decibels::gainToDecibels(magnitudes[i]) + additiveDB;
                float y = jlimit((float) yMin, (float) yMax + OH + 1, dbToYFloat(db));
                float x = xMin + i;
                magnitude.lineTo(x, y);
            }

            g.setColour(handle.colour.withMultipliedAlpha(0.5f));
            g.strokePath(magnitude, PathStrokeType(isActive ? 2.5f : 0.9f));
            
            magnitude.lineTo(xMax, yZero);
            magnitude.lineTo(xMin, yZero);
            magnitude.closeSubPath();
            g.setColour(handle.colour.withMultipliedAlpha(0.1f));
            g.fillPath(magnitude);
            
            float multGain = (handle.overrideGain != nullptr) ? *handle.overrideGain : Decibels::decibelsToGain(additiveDB);
            //overall magnitude update
            if (filtersAreParallel)
            {
                for (int i = 0; i < numPixels; ++i)
                {
                    complexMagnitudes.setUnchecked(i, complexMagnitudes[i] + std::polar(magnitudes[i]*multGain, phases[i])); //*addGain
                }
            }
            else
            {
                for (int i = 0; i < numPixels; ++i)
                {
                    float dB = Decibels::gainToDecibels(magnitudes[i]*multGain);
                    allMagnitudesInDb.setUnchecked(i, allMagnitudesInDb[i] + dB);
                }
            }
        }
        
        if (filtersAreParallel)
            for (int i = 0; i < numPixels; ++i)
            {
                float dB = Decibels::gainToDecibels(std::abs(complexMagnitudes[i]));
                allMagnitudesInDb.setUnchecked(i, allMagnitudesInDb[i] + dB);
            }

        
        //all magnitudes combined
        magnitude.clear();
        magnitude.startNewSubPath(xMin, jlimit((float) yMin, (float) yMax + OH + 1, dbToYFloat(allMagnitudesInDb[0])));
        
        for (int x = xMin + 1; x<=xMax; ++x)
        {
            magnitude.lineTo(x, jlimit((float) yMin, (float) yMax + OH + 1, dbToYFloat(allMagnitudesInDb[x-xMin])));
        }
       // g.setColour(Colours::white);
		g.setColour(Colour(137, 176, 196));
        g.strokePath(magnitude, PathStrokeType(1.5f));
        
        magnitude.lineTo(xMax, yZero);
        magnitude.lineTo(xMin, yZero);
        magnitude.closeSubPath();
		g.setColour(Colour(137, 176, 196).withMultipliedAlpha(0.1f));
		//g.setColour(Colours::white.withMultipliedAlpha(0.1f));
        g.fillPath(magnitude);
        
        //int size = elements.size();
        //for (int i = 0; i < size; ++i) {
        //    FilterWithSlidersAndColour& handle(elements.getReference(i));
        //    float circX = handle.frequencySlider == nullptr ? hzToX(s.fMin) : hzToX(handle.frequencySlider->getValue());
        //    float circY;
        //    if (!s.gainHandleLin)
        //        circY = handle.gainSlider == nullptr ? dbToY(0.0f) : dbToY(handle.gainSlider->getValue());
        //    else
        //        circY = handle.gainSlider == nullptr ? dbToY(0.0f) : dbToY(Decibels::gainToDecibels (handle.gainSlider->getValue()));
        //    g.setColour(Colour(0xFF191919));
        //    g.drawEllipse(circX - 5.0f, circY - 5.0f , 10.0f, 10.0f, 3.0f);
        //    
        //    g.setColour(handle.colour);
        //    g.drawEllipse(circX - 5.0f, circY - 5.0f , 10.0f, 10.0f, 1.0f);
        //    g.setColour(activeElem == i ? handle.colour : handle.colour.withSaturation(0.2));
        //    g.fillEllipse(circX - 5.0f, circY - 5.0f , 10.0f, 10.0f);
        //}
    };
    
    int dbToY(float db)
    {
        int ypos = dbToYFloat(db);
        return ypos;
    }
    
    float dbToYFloat(float db)
    {
        float height = (float) getHeight() - mB - mT;
        float dyn = s.dbMax - s.dbMin;
        float ypos = height * (1.f - (db - s.dbMin) / dyn) + mT;
        return ypos;
    }
    
    float yToDb (float y)
    {
        float height = (float) getHeight() - mB - mT;
        float dyn = s.dbMax - s.dbMin;
        float db = (s.dbMax - (y - mT) / height * dyn);
        return db;
    }
    
    int hzToX(float hz)
    {
        float width = (float) getWidth() - mL - mR;
        int xpos = mL + width * (log(hz/s.fMin) / log(s.fMax/s.fMin));
        return xpos;
    }
    
    float xToHz(int x)
    {
        float width = (float) getWidth() - mL - mR;
        return s.fMin * powf ((s.fMax / s.fMin), ((x - mL) / width));
    }

    void setSampleRate(int newSampleRate) {
        sampleRate = newSampleRate;
    }
    
    void setOverallGain(float newGain) {
        float gainInDb = Decibels::gainToDecibels(newGain);
        if (overallGainInDb != gainInDb)
        {
            overallGainInDb = gainInDb;
            repaint();
        }
    }
    
    void setOverallGainInDecibels(float newGainInDecibels) {
        if (overallGainInDb != newGainInDecibels)
        {
            overallGainInDb = newGainInDecibels;
            repaint();
        }
    }

    
    void resized() override {
        int xMin = hzToX(s.fMin);
        int xMax = hzToX(s.fMax);
        numPixels = xMax - xMin + 1;
        
        frequencies.resize(numPixels);
        for (int i = 0; i < numPixels; ++i)
            frequencies.set(i, xToHz(xMin + i));

        allMagnitudesInDb.resize(numPixels);
        magnitudes.resize(numPixels);
        phases.resize(numPixels);
        complexMagnitudes.resize(numPixels);
        
        const float width = getWidth() - mL - mR;
        dbGridPath.clear();
        
        float dyn = s.dbMax - s.dbMin;
        int numgridlines = dyn/s.gridDiv+1;
        
        for (int i=0; i < numgridlines; i++)
        {
            float db_val = s.dbMax - i * s.gridDiv;
            
            int ypos = dbToY(db_val);

            dbGridPath.startNewSubPath(mL-OH, ypos);
            dbGridPath.lineTo(mL + width+OH, ypos);
        }
        
        hzGridPath.clear();
        hzGridPathBold.clear();
        
        for (float f=s.fMin; f <= s.fMax; f += powf(10, floorf(log10(f)))) {
            int xpos = hzToX(f);
            
            if ((f == 20) || (f == 50) || (f == 100) || (f == 500) || (f == 1000) || (f == 5000) || (f == 10000) || (f == 20000))
            {
                hzGridPathBold.startNewSubPath(xpos, dbToY(s.dbMax)-OH);
                hzGridPathBold.lineTo(xpos, dbToY(s.dbMin)+OH);
                
            } else
            {
                hzGridPath.startNewSubPath(xpos, dbToY(s.dbMax)-OH);
                hzGridPath.lineTo(xpos, dbToY(s.dbMin)+OH);
            }
        }
    }
    
    void addCoefficients(dsp::FIR::Coefficients<float>::Ptr* newCoeffs, Colour newColourForCoeffs, float* overrideLinearGain = nullptr)
    {
        elements.add({newCoeffs, newColourForCoeffs, overrideLinearGain});
    }
    
    void setParallel (bool shouldBeParallel)
    {
        filtersAreParallel = shouldBeParallel;
    }
    
private:
    
    bool filtersAreParallel = false;
    float overallGainInDb;
    
    int sampleRate;
    
    int activeElem = 0;

    
    Settings s;
    Path dbGridPath;
    Path hzGridPath;
    Path hzGridPathBold;
    
    Array<double> frequencies;
    Array<double> magnitudes;
    Array<double> phases;
    int numPixels;
    
    Array<std::complex<double>> complexMagnitudes;
    Array<FilterWithSlidersAndColour> elements;
    Array<float> allMagnitudesInDb;
};
