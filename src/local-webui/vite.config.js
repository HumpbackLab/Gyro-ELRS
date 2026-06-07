import { defineConfig } from 'vite';

function sanitizeTarget(rawTarget) {
  if (!rawTarget) return null;
  try {
    const url = new URL(rawTarget);
    if (!/^https?:$/.test(url.protocol)) return null;
    url.hash = '';
    return url;
  } catch {
    return null;
  }
}

function buildTarget(requestUrl, req) {
  const requestTarget = req.headers['x-elrs-target'];
  const headerTarget = Array.isArray(requestTarget) ? requestTarget[0] : requestTarget;
  const queryTarget = new URL(requestUrl, 'http://127.0.0.1').searchParams.get('target');
  return sanitizeTarget(headerTarget || queryTarget);
}

export default defineConfig({
  base: './',
  plugins: [{
    name: 'elrs-local-proxy',
    configureServer(server) {
      server.middlewares.use('/__elrs_proxy__', async (req, res) => {
        const target = buildTarget(req.url, req);
        if (!target) {
          res.statusCode = 400;
          res.setHeader('Content-Type', 'application/json');
          res.end(JSON.stringify({status: 'error', msg: 'Missing or invalid target URL'}));
          return;
        }

        const requestUrl = new URL(req.url, 'http://127.0.0.1');
        target.pathname = requestUrl.pathname.replace(/^\/__elrs_proxy__/, '') || '/';
        target.search = requestUrl.searchParams.has('target')
          ? `?${new URLSearchParams([...requestUrl.searchParams.entries()].filter(([key]) => key !== 'target')).toString()}`
          : requestUrl.search;

        const headers = new Headers();
        for (const [key, value] of Object.entries(req.headers)) {
          if (value === undefined) continue;
          if (key === 'host' || key === 'connection' || key === 'content-length' || key === 'x-elrs-target') continue;
          if (Array.isArray(value)) {
            for (const item of value) headers.append(key, item);
          } else {
            headers.set(key, value);
          }
        }

        const body = req.method === 'GET' || req.method === 'HEAD'
          ? undefined
          : await new Promise((resolve, reject) => {
              const chunks = [];
              req.on('data', (chunk) => chunks.push(chunk));
              req.on('end', () => resolve(Buffer.concat(chunks)));
              req.on('error', reject);
            });

        try {
          const upstream = await fetch(target, {
            method: req.method,
            headers,
            body,
            duplex: 'half',
          });

          res.statusCode = upstream.status;
          upstream.headers.forEach((value, key) => {
            if (key === 'transfer-encoding' || key === 'connection') return;
            res.setHeader(key, value);
          });
          const payload = Buffer.from(await upstream.arrayBuffer());
          res.end(payload);
        } catch (error) {
          res.statusCode = 502;
          res.setHeader('Content-Type', 'application/json');
          res.end(JSON.stringify({status: 'error', msg: error.message || String(error)}));
        }
      });
    },
  }],
  server: {
    host: '127.0.0.1',
    port: 5173,
    strictPort: false,
  },
});
