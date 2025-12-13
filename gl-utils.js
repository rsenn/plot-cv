import { RESIZABLE, SAMPLES, CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, OPENGL_FORWARD_COMPAT, OPENGL_PROFILE, OPENGL_CORE_PROFILE, Window } from 'glfw';

export class GLFW extends Window {
  constructor(w, h, opts) {
    const { resizable = false, samples = 4, contextVersionMajor = 3, contextVersionMinor = 2, openglProfile = OPENGL_CORE_PROFILE, openglForwardCompat = true, title = 'OpenGL', ...handlers } = opts;

    const hints = [
      [CONTEXT_VERSION_MAJOR, contextVersionMajor],
      [CONTEXT_VERSION_MINOR, contextVersionMinor],
      [OPENGL_PROFILE, openglProfile],
      [OPENGL_FORWARD_COMPAT, openglForwardCompat],
      [RESIZABLE, resizable],
      [SAMPLES, samples]
    ];

    for(let [prop, value] of hints) Window.hint(prop, value);

    super(w, h, title);
  }
}