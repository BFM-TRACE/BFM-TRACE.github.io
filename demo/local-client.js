// Each page owns a dedicated worker and therefore its own policy/physics state.
// No state or command is sent to a simulation server.
export class LocalSimulation {
  constructor(onProgress = () => {}) {
    this.onProgress = onProgress;
    this.pending = new Map();
    this.nextId = 0;
    this.latest = null;
    this.hidden = false;
  }

  start() {
    if (this.ready) return this.ready;
    this.worker = new Worker(new URL('./local-worker.js', import.meta.url), {type:'module'});
    this.worker.onmessage = ({data}) => {
      if (data.type === 'progress') this.onProgress(data.message);
      if (data.type === 'state') this.latest = data.state;
      if (data.type === 'fatal') this.fail(new Error(data.error));
      const pending = this.pending.get(data.id);
      if (!pending) return;
      this.pending.delete(data.id); clearTimeout(pending.timer);
      if (data.error) pending.reject(new Error(data.error));
      else pending.resolve(data.result);
    };
    this.worker.onerror = (event) => this.fail(new Error(event.message || 'The browser simulation worker stopped.'));
    this.worker.postMessage({type:'visibility',payload:{hidden:this.hidden}});
    this.ready = this.rpc('init', {}, 180000).catch(error=>{this.fail(error);throw error;});
    return this.ready;
  }

  fail(error) {
    this.error = error;
    if (this.latest) this.latest = {...this.latest,status:'error',error:error.message};
    for (const item of this.pending.values()) {clearTimeout(item.timer); item.reject(error);}
    this.pending.clear();
    this.worker?.terminate();
  }

  rpc(type, payload, timeout = 15000) {
    if (this.error) return Promise.reject(this.error);
    return new Promise((resolve,reject) => {
      const id = ++this.nextId;
      const timer = setTimeout(() => {this.pending.delete(id); reject(new Error('The browser simulation did not respond. Reload to retry.'));},timeout);
      this.pending.set(id,{resolve,reject,timer});
      this.worker.postMessage({id,type,payload});
    });
  }

  async request(path, payload) {
    await this.start();
    if (this.error) throw this.error;
    if (path === 'command') return this.rpc('command',payload);
    if (!this.latest) throw new Error('Waiting for the first local physics state.');
    return this.latest;
  }

  suspend(hidden) {
    this.hidden = hidden;
    if (this.worker && !this.error) this.worker.postMessage({type:'visibility',payload:{hidden}});
  }

  restart() {
    this.worker?.terminate(); this.worker=null;
    this.ready=null; this.error=null; this.latest=null;
  }

  dispose() {this.fail(new Error('The simulation was closed.'));}
}
