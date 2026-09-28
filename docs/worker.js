/* Runs the vectorizer off the main thread so the page stays responsive. */
importScripts('engine.js');

self.onmessage = (e) => {
  const { rgba, width, height } = e.data;
  try {
    const result = self.LogoVectorizer.vectorizeRGBA(new Uint8ClampedArray(rgba), width, height,
      (msg) => self.postMessage({ type: 'progress', msg }));
    self.postMessage({ type: 'done', result });
  } catch (err) {
    self.postMessage({ type: 'error', msg: (err && err.message) || String(err) });
  }
};
