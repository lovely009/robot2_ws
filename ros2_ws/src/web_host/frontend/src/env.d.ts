/// <reference types="vite/client" />

declare namespace ROSLIB {
  class Ros {
    constructor(options: { url: string });
    on(event: string, callback: (...args: any[]) => void): void;
    close(): void;
  }
  class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
      throttle_rate?: number;
      queue_length?: number;
      qos?: Record<string, unknown>;
    });
    subscribe(callback: (message: any) => void): void;
    unsubscribe(): void;
    advertise(): void;
    unadvertise(): void;
    publish(message: Message): void;
  }
  class Message {
    constructor(values: Record<string, any>);
    [key: string]: any;
  }
  class Service {
    constructor(options: { ros: Ros; name: string; serviceType: string });
    callService(
      request: Message,
      callback: (result: any) => void,
      failedCallback?: (error: any) => void,
    ): void;
  }
}

declare module "three" {
  export const Object3D: any;
  export class Vector2 {
    x: number;
    y: number;
    constructor(x?: number, y?: number);
  }
  export class Vector3 {
    x: number;
    y: number;
    z: number;
    constructor(x?: number, y?: number, z?: number);
    set(x: number, y: number, z: number): this;
    copy(v: Vector3): this;
    clone(): Vector3;
    normalize(): this;
    lengthSq(): number;
    length(): number;
    multiplyScalar(s: number): this;
    applyQuaternion(q: Quaternion): this;
    add(v: Vector3): this;
  }
  export class Quaternion {
    x: number;
    y: number;
    z: number;
    w: number;
    constructor(x?: number, y?: number, z?: number, w?: number);
    copy(q: Quaternion): this;
    clone(): Quaternion;
    invert(): this;
    fromArray(arr: number[]): this;
    multiply(q: Quaternion): this;
    setFromRotationMatrix(m: Matrix4): this;
    setFromUnitVectors(vFrom: Vector3, vTo: Vector3): this;
  }
  export class Matrix3 {
    set(...n: number[]): this;
    transpose(): this;
  }
  export class Matrix4 {
    setFromMatrix3(m: Matrix3): this;
  }
  export class Color {
    constructor(color?: number | string);
    set(color: number | string): this;
    getHexString(): string;
  }
  export class Box3 {
    min: Vector3;
    max: Vector3;
    constructor();
    setFromObject(object: any, precise?: boolean): this;
    isEmpty(): boolean;
    getCenter(target: Vector3): Vector3;
    getSize(target: Vector3): Vector3;
    union(box: Box3): this;
    copy(box: Box3): this;
    applyMatrix4(m: Matrix4): this;
  }
  export class Scene {
    background: Color | null;
    children: any[];
    add(...objects: any[]): this;
    remove(...objects: any[]): this;
    traverse(cb: (obj: any) => void): void;
  }
  export class Group {
    position: Vector3;
    quaternion: Quaternion;
    add(...objects: any[]): this;
    remove(...objects: any[]): this;
    updateMatrixWorld(force?: boolean): void;
  }
  export class PerspectiveCamera {
    up: Vector3;
    position: Vector3;
    aspect: number;
    near: number;
    far: number;
    constructor(fov?: number, aspect?: number, near?: number, far?: number);
    updateProjectionMatrix(): void;
  }
  export class WebGLRenderer {
    domElement: HTMLCanvasElement;
    constructor(params?: Record<string, any>);
    setPixelRatio(value: number): void;
    setSize(width: number, height: number, updateStyle?: boolean): void;
    getSize(target: Vector2): Vector2;
    render(scene: Scene, camera: PerspectiveCamera): void;
    setAnimationLoop(callback: ((time: number) => void) | null): void;
    dispose(): void;
  }
  export class HemisphereLight {
    position: Vector3;
    constructor(skyColor?: number, groundColor?: number, intensity?: number);
  }
  export class DirectionalLight {
    position: Vector3;
    constructor(color?: number, intensity?: number);
  }
  export class GridHelper {
    position: Vector3;
    rotation: { x: number; y: number; z: number };
    geometry: any;
    material: any;
    constructor(size?: number, divisions?: number, color1?: number, color2?: number);
  }
  export class MeshBasicMaterial {
    constructor(params?: Record<string, any>);
    dispose(): void;
  }
  export class LineBasicMaterial {
    color: Color;
    depthTest: boolean;
    depthWrite: boolean;
    constructor(params?: Record<string, any>);
    dispose(): void;
  }
  export class PointsMaterial {
    constructor(params?: Record<string, any>);
    dispose(): void;
  }
  export class SphereGeometry {
    constructor(radius?: number, widthSegments?: number, heightSegments?: number);
  }
  export class BufferGeometry {
    attributes: Record<string, any>;
    boundingBox: Box3 | null;
    setAttribute(name: string, attribute: any): this;
    setFromPoints(points: Vector3[]): this;
    computeBoundingSphere(): void;
    computeBoundingBox(): void;
    dispose(): void;
  }
  export class BufferAttribute {
    needsUpdate: boolean;
    constructor(array: ArrayLike<number>, itemSize: number);
    setXYZ(index: number, x: number, y: number, z: number): this;
  }
  export class Mesh {
    userData: Record<string, any>;
    material: any;
    geometry: any;
    children: any[];
    scale: { setScalar(s: number): void; x: number };
    constructor(geometry?: any, material?: any);
    add(...objects: any[]): this;
  }
  export class LineSegments {
    visible: boolean;
    geometry: BufferGeometry;
    material: any;
    constructor(geometry?: BufferGeometry, material?: any);
  }
  export class Points {
    visible: boolean;
    geometry: BufferGeometry;
    material: any;
    constructor(geometry?: BufferGeometry, material?: any);
  }
  export class LineLoop {
    quaternion: Quaternion;
    material: any;
    renderOrder: number;
    constructor(geometry?: BufferGeometry, material?: any);
  }
  export class ArrowHelper {
    line: any;
    cone: any;
    constructor(dir: Vector3, origin: Vector3, length: number, color: number, headLength?: number, headWidth?: number);
    setColor(color: number | string): void;
  }
  export class Raycaster {
    setFromCamera(coords: Vector2, camera: PerspectiveCamera): void;
    intersectObjects(objects: any[], recursive?: boolean): any[];
  }
  export class LoadingManager {
    onProgress: ((url: string, loaded: number, total: number) => void) | null;
    onLoad: (() => void) | null;
    setURLModifier(callback: (url: string) => string): void;
  }
}

declare module "three/examples/jsm/controls/OrbitControls.js" {
  export class OrbitControls {
    enableDamping: boolean;
    dampingFactor: number;
    target: any;
    constructor(object: any, domElement?: HTMLElement);
    update(): boolean;
    dispose(): void;
  }
}

declare module "/vendor/URDFLoader.js" {
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
}

interface Window {
  __workbench?: Record<string, any>;
}
