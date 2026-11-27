export default class URDFLoader {
  packages: Record<string, string>;
  parseCollision: boolean;
  loadMeshCb: ((path: string, manager: any, material: any, done: (obj: any, err?: any) => void) => void) | null;
  constructor(manager?: any);
  load(
    url: string,
    onComplete: (robot: any) => void,
    onProgress?: (url: string, loaded: number, total: number) => void,
    onError?: (err: any) => void,
  ): void;
  parse(doc: Document | string, workingPath?: string): any;
  defaultMeshLoader(path: string, manager: any, material: any, done: (obj: any, err?: any) => void): void;
}
