/* scales generated in supercolider
 * Scale.directory; (36,50,
 * Scale.bhairav.ratios *  80.midicps; (36,48,60,72. ....)
 * does not account for alt tuning, etc.
 * a = Scale.neapolitanMinor.ratios ;///* [36,48,60,72,84].midicps ; 
 * [36,48,60,72,84].do ( { |midi| (midi.midicps * a).postln }) ; will produce all
)
 */
 
// Bank 1 ragas

//nahawand raga up and down
const float Nahawand[32] PROGMEM = {
  65.40639132515, 73.416191979318, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 123.47082531372, 116.54094037925,
  130.8127826503, 146.83238395864, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 246.94165062743, 233.08188075851,
  261.6255653006, 293.66476791727, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 493.88330125487, 466.16376151701, 
  523.2511306012, 587.32953583454, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 987.76660250974, 932.32752303403,
};

//nahawand raga up and down
const float Bayati[32] PROGMEM = {
69.295657744138, 77.781745930394, 82.406889228065, 92.498605677695,
65.40639132515, 71.326175505781, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 116.54094037925,
130.8127826503, 142.65235101156, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 233.08188075851,
261.6255653006, 285.30470202312, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 466.16376151701,
523.2511306012, 570.60940404625, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 932.32752303403,
};


const float NawaAthar[32] PROGMEM = {
73.41619197925, 77.781745930394, 82.406889228065, 97.998858995188,
103.82617439499, 116.54094037947, 123.47082531395, 146.8323839585, 155.56349186079, 164.81377845613, 195.99771799038,
207.65234878997, 233.08188075894, 246.94165062789, 293.664767917, 311.12698372158, 329.62755691226, 391.99543598075,
415.30469757995, 466.16376151787, 493.88330125578, 587.329535834, 622.25396744316, 659.25511382452, 783.99087196151,
523.2511306012, 587.32953583454, 622.25396744373, 739.98884542224, 783.99087196223, 830.60939515835,  987.76660250974 
};

const float Farafahza[32] PROGMEM = {
65.40639132515, 73.416191979318, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 116.54094037925 ,
116.54094037952, 130.81278265024, 138.59131548834, 155.56349186086, 174.61411571622, 184.99721135548, 207.65234878949 ,
233.08188075904, 261.62556530048, 277.18263097668, 311.12698372172, 349.22823143244, 369.99442271095, 415.30469757899 ,
466.16376151809, 523.25113060096, 554.36526195336, 622.25396744344, 698.45646286488, 739.9888454219, 830.60939515797 ,
932.32752303618, 1046.5022612019, 1108.7305239067, 1244.5079348869
};

const float Nikriz[32] PROGMEM = {
65.40639132515, 73.416191979318, 77.781745930466, 92.49860567778, 97.998858995279, 109.99999999977, 116.54094037925,
130.8127826503, 146.83238395864, 155.56349186093, 184.99721135556, 195.99771799056, 219.99999999954, 233.08188075851,
261.6255653006, 293.66476791727, 311.12698372187, 369.99442271112, 391.99543598112, 439.99999999908, 466.16376151701,
523.2511306012, 587.32953583454, 622.25396744373, 739.98884542224, 783.99087196223, 879.99999999817, 932.32752303403,
1046.5022612024, 1174.6590716691, 1244.5079348875, 1479.9776908445,
};

const float Bhairav[32] PROGMEM = {
 65.40639132515, 69.295657744202, 82.406889228141, 87.30705785815, 97.998858995279, 103.82617439479, 123.47082531372 ,
 146.8323839587, 155.563491861, 184.99721135565, 195.99771799065, 219.99999999964, 233.08188075861, 277.18263097617 ,
 293.66476791741, 311.12698372201, 369.99442271129, 391.9954359813, 439.99999999929, 466.16376151723, 554.36526195234 ,
 587.32953583482, 622.25396744402, 739.98884542258, 783.99087196259, 879.99999999858, 932.32752303446, 1108.7305239047 ,
 830.60939515989, 879.9999999998, 1046.5022612014, 1108.7305239062};



// dead Europeans
const float NeapolitanMinor[32] PROGMEM = {
65.40639132515, 69.295657744202, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 123.47082531372 ,
130.8127826503, 138.5913154884, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 246.94165062743 ,
261.6255653006, 277.18263097681, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 493.88330125487,
523.2511306012, 554.36526195362, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 987.76660250974,
1046.5022612024, 1108.7305239072, 1244.5079348875, 1396.9129257304
 };
 
const float MelodicMinor[32] PROGMEM = {
65.40639132515, 73.416191979318, 77.781745930466, 87.30705785815, 97.998858995279, 109.99999999977, 123.47082531372 ,
130.8127826503, 146.83238395864, 155.56349186093, 174.6141157163, 195.99771799056, 219.99999999954, 246.94165062743 ,
261.6255653006, 293.66476791727, 311.12698372187, 349.2282314326, 391.99543598112, 439.99999999908, 493.88330125487 ,
523.2511306012, 587.32953583454, 622.25396744373, 698.4564628652, 783.99087196223, 879.99999999817, 987.76660250974 ,
1046.5022612024, 1174.6590716691, 1244.5079348875, 1396.9129257304, 
};

const float PhrygianFreq[32] PROGMEM = {
65.40639132515, 69.295657744202, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 116.54094037925,
130.8127826503, 138.5913154884, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 233.08188075851,
261.6255653006, 277.18263097681, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 466.16376151701,
523.2511306012, 554.36526195362, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 932.32752303403,
1046.5022612024, 1108.7305239072, 1244.5079348875, 1396.9129257304
};

const float RomanMinor[32] PROGMEM = {
65.40639132515, 73.416191979318, 77.781745930466, 92.49860567778, 97.998858995279, 109.99999999977, 116.54094037925 ,
130.8127826503, 146.83238395864, 155.56349186093, 184.99721135556, 195.99771799056, 219.99999999954, 233.08188075851 ,
261.6255653006, 293.66476791727, 311.12698372187, 369.99442271112, 391.99543598112, 439.99999999908, 466.16376151701 ,
523.2511306012, 587.32953583454, 622.25396744373, 739.98884542224, 783.99087196223, 879.99999999817, 932.32752303403 ,
1046.5022612024, 1174.6590716691, 1244.5079348875, 1479.9776908445
};
const float Spanish[32] PROGMEM = {
65.40639132515, 69.295657744202, 82.406889228141, 87.30705785815, 97.998858995279, 103.82617439479, 116.54094037925 ,
130.8127826503, 138.5913154884, 164.81377845628, 174.6141157163, 195.99771799056, 207.65234878959, 233.08188075851 ,
261.6255653006, 277.18263097681, 329.62755691257, 349.2282314326, 391.99543598112, 415.30469757918, 466.16376151701 ,
523.2511306012, 554.36526195362, 659.25511382513, 698.4564628652, 783.99087196223, 830.60939515835, 932.32752303403 ,
1046.5022612024, 1108.7305239072, 1318.5102276503, 1396.9129257304 ,
};
const float LydianMinor[32] PROGMEM = {
65.40639132515, 73.416191979318, 82.406889228141, 92.49860567778, 97.998858995279, 103.82617439479, 116.54094037925 ,
130.8127826503, 146.83238395864, 164.81377845628, 184.99721135556, 195.99771799056, 207.65234878959, 233.08188075851 ,
261.6255653006, 293.66476791727, 329.62755691257, 369.99442271112, 391.99543598112, 415.30469757918, 466.16376151701 ,
523.2511306012, 587.32953583454, 659.25511382513, 739.98884542224, 783.99087196223, 830.60939515835, 932.32752303403 ,
1046.5022612024, 1174.6590716691, 1318.5102276503, 1479.9776908445,
};

// madmen and mice
const float Partch1[32] PROGMEM = {
49.054793493783, 57.230592409378 ,
65.40639132515, 73.582190240759, 81.757989156364, 89.933788071966, 98.109586987565, 114.46118481876 ,
130.8127826503, 147.16438048152, 163.51597831273, 179.86757614393, 196.21917397513, 228.92236963751 ,
261.6255653006, 294.32876096303, 327.03195662546, 359.73515228786, 392.43834795026, 457.84473927502 ,
523.2511306012, 588.65752192607, 654.06391325091, 719.47030457573, 784.87669590052, 915.68947855005 ,
1046.5022612024, 1177.3150438521, 1308.1278265018, 1438.9406091515, 1569.753391801, 1831.3789571001
};

const float Suznak[32] PROGMEM = {
65.40639132515, 73.416191979318, 80.060925056256, 87.30705785815, 97.998858995279, 103.82617439479, 123.47082531372 ,
130.8127826503, 146.83238395864, 160.12185011251, 174.6141157163, 195.99771799056, 207.65234878959, 246.94165062743 ,
261.6255653006, 293.66476791727, 320.24370022502, 349.2282314326, 391.99543598112, 415.30469757918, 493.88330125487 ,
523.2511306012, 587.32953583454, 640.48740045004, 698.4564628652, 783.99087196223, 830.60939515835, 987.76660250974 ,
1046.5022612024, 1174.6590716691, 1280.9748009001, 1396.9129257304
};
const float Zamzam[32] PROGMEM = {
65.40639132515, 69.295657744202, 77.781745930466, 82.406889228141, 97.998858995279, 103.82617439479, 116.54094037925 ,
130.8127826503, 138.5913154884, 155.56349186093, 164.81377845628, 195.99771799056, 207.65234878959, 233.08188075851 ,
261.6255653006, 277.18263097681, 311.12698372187, 329.62755691257, 391.99543598112, 415.30469757918, 466.16376151701 ,
523.2511306012, 554.36526195362, 622.25396744373, 659.25511382513, 783.99087196223, 830.60939515835, 932.32752303403 ,
1046.5022612024, 1108.7305239072, 1244.5079348875, 1318.5102276503
};
const float KijazKarKurd[32] PROGMEM = {
65.40639132515, 69.295657744202, 82.406889228141, 87.30705785815, 97.998858995279, 103.82617439479, 123.47082531372 ,
130.8127826503, 138.5913154884, 164.81377845628, 174.6141157163, 195.99771799056, 207.65234878959, 246.94165062743 ,
261.6255653006, 277.18263097681, 329.62755691257, 349.2282314326, 391.99543598112, 415.30469757918, 493.88330125487 ,
523.2511306012, 554.36526195362, 659.25511382513, 698.4564628652, 783.99087196223, 830.60939515835, 987.76660250974 ,
1046.5022612024, 1108.7305239072, 1318.5102276503, 1396.9129257304
};
const float RomanianMinor[32] PROGMEM = {
65.40639132515, 73.416191979318, 77.781745930466, 92.49860567778, 97.998858995279, 109.99999999977, 116.54094037925 ,
130.8127826503, 146.83238395864, 155.56349186093, 184.99721135556, 195.99771799056, 219.99999999954, 233.08188075851 ,
261.6255653006, 293.66476791727, 311.12698372187, 369.99442271112, 391.99543598112, 439.99999999908, 466.16376151701 ,
523.2511306012, 587.32953583454, 622.25396744373, 739.98884542224, 783.99087196223, 879.99999999817, 932.32752303403 ,
1046.5022612024, 1174.6590716691, 1244.5079348875, 1479.9776908445
};
const float Enigmatic[32] PROGMEM = {
65.40639132515, 69.295657744202, 82.406889228141, 92.49860567778, 103.82617439479, 116.54094037925, 123.47082531372 ,
130.8127826503, 138.5913154884, 164.81377845628, 184.99721135556, 207.65234878959, 233.08188075851, 246.94165062743 ,
261.6255653006, 277.18263097681, 329.62755691257, 369.99442271112, 415.30469757918, 466.16376151701, 493.88330125487 ,
523.2511306012, 554.36526195362, 659.25511382513, 739.98884542224, 830.60939515835, 932.32752303403, 987.76660250974 ,
1046.5022612024, 1108.7305239072, 1318.5102276503, 1479.9776908445
};
