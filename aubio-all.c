#include "/home/roman/Sources/aubio/src/io/sink_wavwrite.c"
#include "/home/roman/Sources/aubio/src/io/sink_apple_audio.c"
#include "/home/roman/Sources/aubio/src/io/ioutils.c"
#include "/home/roman/Sources/aubio/src/io/source_apple_audio.c"
#include "/home/roman/Sources/aubio/src/io/source.c"
#include "/home/roman/Sources/aubio/src/io/utils_apple_audio.c"
#include "/home/roman/Sources/aubio/src/io/source_wavread.c"
#include "/home/roman/Sources/aubio/src/io/source_avcodec.c"
#include "/home/roman/Sources/aubio/src/io/sink.c"
#include "/home/roman/Sources/aubio/src/io/audio_unit.c"
#include "/home/roman/Sources/aubio/src/io/source_sndfile.c"
#include "/home/roman/Sources/aubio/src/io/sink_sndfile.c"
#include "/home/roman/Sources/aubio/src/musicutils.c"
#include "/home/roman/Sources/aubio/src/lvec.c"
#include "/home/roman/Sources/aubio/src/onset/onset.c"
#include "/home/roman/Sources/aubio/src/onset/peakpicker.c"
#include "/home/roman/Sources/aubio/src/vecutils.c"
#include "/home/roman/Sources/aubio/src/cvec.c"
#include "/home/roman/Sources/aubio/src/fmat.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchyinfft.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchfcomb.c"
#include "/home/roman/Sources/aubio/src/pitch/pitch.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchschmitt.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchyin.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchspecacf.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchyinfast.c"
#include "/home/roman/Sources/aubio/src/pitch/pitchmcomb.c"
#include "/home/roman/Sources/aubio/src/notes/notes.c"
#include "/home/roman/Sources/aubio/src/utils/windll.c"
#include "/home/roman/Sources/aubio/src/utils/scale.c"
#include "/home/roman/Sources/aubio/src/utils/log.c"
#include "/home/roman/Sources/aubio/src/utils/hist.c"
#include "/home/roman/Sources/aubio/src/utils/parameter.c"
#include "/home/roman/Sources/aubio/src/spectral/filterbank_mel.c"
#include "/home/roman/Sources/aubio/src/spectral/fft.c"
#include "/home/roman/Sources/aubio/src/spectral/filterbank.c"
#include "/home/roman/Sources/aubio/src/spectral/tss.c"
#include "/home/roman/Sources/aubio/src/spectral/dct_plain.c"
#include "/home/roman/Sources/aubio/src/spectral/dct_fftw.c"
#include "/home/roman/Sources/aubio/src/spectral/dct_ipp.c"
#include "/home/roman/Sources/aubio/src/spectral/awhitening.c"
#include "/home/roman/Sources/aubio/src/spectral/specdesc.c"
#include "/home/roman/Sources/aubio/src/spectral/phasevoc.c"
#include "/home/roman/Sources/aubio/src/spectral/mfcc.c"
#include "/home/roman/Sources/aubio/src/spectral/statistics.c"
#include "/home/roman/Sources/aubio/src/spectral/dct.c"
#include "/home/roman/Sources/aubio/src/spectral/dct_accelerate.c"
#include "/home/roman/Sources/aubio/src/spectral/dct_ooura.c"
#include "/home/roman/Sources/aubio/src/spectral/ooura_fft8g.c"
#include "/home/roman/Sources/aubio/src/fvec.c"
#include "/home/roman/Sources/aubio/src/temporal/filter.c"
#include "/home/roman/Sources/aubio/src/temporal/c_weighting.c"
#include "/home/roman/Sources/aubio/src/temporal/biquad.c"
#include "/home/roman/Sources/aubio/src/temporal/a_weighting.c"
#include "/home/roman/Sources/aubio/src/temporal/resampler.c"
#include "/home/roman/Sources/aubio/src/synth/wavetable.c"
#include "/home/roman/Sources/aubio/src/synth/sampler.c"
#include "/home/roman/Sources/aubio/src/mathutils.c"
#include "/home/roman/Sources/aubio/src/tempo/beattracking.c"
#include "/home/roman/Sources/aubio/src/tempo/tempo.c"
