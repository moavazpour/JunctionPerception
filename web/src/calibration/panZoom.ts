/** 2D view of a canvas: content coordinates <-> screen pixels, with optional y-up content. */
export class View2D {
  scale = 1;
  offsetX = 0;
  offsetY = 0;

  constructor(readonly yUp: boolean) {}

  toScreen(x: number, y: number): [number, number] {
    return [x * this.scale + this.offsetX, (this.yUp ? -y : y) * this.scale + this.offsetY];
  }

  toContent(sx: number, sy: number): [number, number] {
    const x = (sx - this.offsetX) / this.scale;
    const y = (sy - this.offsetY) / this.scale;
    return [x, this.yUp ? -y : y];
  }

  /** Fits the box [minX, minY]..[maxX, maxY] into a canvas of the given size. */
  fit(minX: number, minY: number, maxX: number, maxY: number, width: number, height: number) {
    this.scale = Math.min(width / (maxX - minX), height / (maxY - minY));
    const [cx, cy] = [(minX + maxX) / 2, (minY + maxY) / 2];
    this.offsetX = width / 2 - cx * this.scale;
    this.offsetY = height / 2 - (this.yUp ? -cy : cy) * this.scale;
  }

  /** Zooms by `factor` keeping the content under the screen point fixed. */
  zoomAt(sx: number, sy: number, factor: number): void {
    const [x, y] = this.toContent(sx, sy);
    this.scale *= factor;
    const [nx, ny] = this.toScreen(x, y);
    this.offsetX += sx - nx;
    this.offsetY += sy - ny;
  }
}

const DRAG_THRESHOLD_PX = 4;
const ZOOM_STEP = 1.15;

/**
 * Canvas with wheel zoom and drag pan; a click without dragging reports the content point.
 * The owner draws in `draw` using the view.
 */
export class PanZoomCanvas {
  readonly view: View2D;
  private readonly context: CanvasRenderingContext2D;
  private press: { x: number; y: number; moved: boolean } | null = null;

  constructor(
    readonly canvas: HTMLCanvasElement,
    yUp: boolean,
    private readonly draw: (context: CanvasRenderingContext2D, view: View2D) => void,
    private readonly onClick: (x: number, y: number, event: MouseEvent) => void,
  ) {
    this.view = new View2D(yUp);
    const context = canvas.getContext('2d');
    if (!context) {
      throw new Error('2D canvas not available');
    }
    this.context = context;
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.view.zoomAt(e.offsetX, e.offsetY, e.deltaY < 0 ? ZOOM_STEP : 1 / ZOOM_STEP);
      this.redraw();
    });
    canvas.addEventListener('pointerdown', (e) => {
      this.press = { x: e.offsetX, y: e.offsetY, moved: false };
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', (e) => {
      if (!this.press) {
        return;
      }
      const dx = e.offsetX - this.press.x;
      const dy = e.offsetY - this.press.y;
      if (this.press.moved || Math.hypot(dx, dy) > DRAG_THRESHOLD_PX) {
        this.press.moved = true;
        this.view.offsetX += dx;
        this.view.offsetY += dy;
        this.press.x = e.offsetX;
        this.press.y = e.offsetY;
        this.redraw();
      }
    });
    canvas.addEventListener('pointerup', (e) => {
      if (this.press && !this.press.moved && e.button === 0) {
        const [x, y] = this.view.toContent(e.offsetX, e.offsetY);
        this.onClick(x, y, e);
      }
      this.press = null;
    });
    new ResizeObserver(() => this.resize()).observe(canvas);
  }

  private resize(): void {
    const ratio = window.devicePixelRatio;
    this.canvas.width = Math.round(this.canvas.clientWidth * ratio);
    this.canvas.height = Math.round(this.canvas.clientHeight * ratio);
    this.redraw();
  }

  redraw(): void {
    const ratio = window.devicePixelRatio;
    this.context.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.context.clearRect(0, 0, this.canvas.clientWidth, this.canvas.clientHeight);
    this.draw(this.context, this.view);
  }
}
