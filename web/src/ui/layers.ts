import type { MapData } from '../map/mapData';
import { attributionLines } from '../map/mapLayers';
import type { BaseLayer, TwinView } from '../twin/twinView';

/** Layer switch (stylized map / orthophoto, buildings) and the attribution of the map sources. */
export function setupLayerControls(view: TwinView, map: MapData): void {
  const controls = document.querySelector<HTMLElement>('#layers');
  const attribution = document.querySelector<HTMLElement>('#attribution');
  if (!controls || !attribution) {
    return;
  }
  controls.hidden = false;
  const buttons = [...controls.querySelectorAll<HTMLButtonElement>('button[data-layer]')];
  for (const button of buttons) {
    const layer = button.dataset['layer'] as BaseLayer;
    button.disabled = layer === 'ortho' && !map.ortho;
    button.addEventListener('click', () => {
      view.showLayer(layer);
      buttons.forEach((b) => b.classList.toggle('active', b === button));
    });
  }
  const buildings = controls.querySelector<HTMLInputElement>('#buildings');
  buildings?.addEventListener('change', () => view.showBuildings(buildings.checked));

  attribution.replaceChildren(
    ...attributionLines(map).map(({ text, url }) => {
      const link = document.createElement('a');
      link.href = url;
      link.target = '_blank';
      link.rel = 'noopener';
      link.textContent = text;
      return link;
    }),
  );
}
