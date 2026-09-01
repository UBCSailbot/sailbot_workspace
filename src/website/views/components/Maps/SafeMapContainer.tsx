'use client';

import {
  createLeafletContext,
  LeafletProvider,
  type LeafletContextInterface,
} from '@react-leaflet/core';
import { Map as LeafletMap, type LatLngExpression, type MapOptions } from 'leaflet';
import React, {
  forwardRef,
  useCallback,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
  type CSSProperties,
  type ReactNode,
  type Ref,
} from 'react';
import { flushSync } from 'react-dom';

type SafeMapContainerProps = MapOptions & {
  center: LatLngExpression;
  zoom: number;
  children?: ReactNode;
  className?: string;
  id?: string;
  style?: CSSProperties;
  whenReady?: () => void;
};

/**
 * Drop-in MapContainer that survives React Strict Mode.
 *
 * react-leaflet v4 gates init on React state (`context === null`), so Strict
 * Mode's attach→detach→attach ref cycle can call L.map() twice on the same
 * node before setState lands. v5 fixed this with a ref guard; we mirror that
 * here so we don't have to bump to React 19 / react-leaflet v5 yet.
 *
 * Teardown also unmounts React children (via flushSync) before Leaflet
 * mutates the container, which avoids `removeChild` null errors on HMR /
 * Strict Mode remounts.
 */
function SafeMapContainerComponent(
  {
    center,
    zoom,
    children,
    className,
    id,
    style,
    whenReady,
    ...options
  }: SafeMapContainerProps,
  forwardedRef: Ref<LeafletMap | null>,
) {
  const [props] = useState({ className, id, style });
  const [context, setContext] = useState<LeafletContextInterface | null>(null);
  const mapInstanceRef = useRef<LeafletMap | undefined>(undefined);

  useImperativeHandle(forwardedRef, () => context?.map ?? null, [context]);

  const destroyMap = useCallback(() => {
    const map = mapInstanceRef.current;
    if (!map) {
      return;
    }
    mapInstanceRef.current = undefined;
    // Drop React-leaflet children while the container DOM is still intact.
    flushSync(() => {
      setContext(null);
    });
    try {
      map.remove();
    } catch {
      // Container may already be gone after Fast Refresh.
    }
  }, []);

  const mapRef = useCallback(
    (node: HTMLDivElement | null) => {
      if (node === null) {
        destroyMap();
        return;
      }

      if (mapInstanceRef.current) {
        return;
      }

      // Strict Mode reappearLayoutEffects can hand back the same DOM node still
      // tagged by Leaflet after the previous fiber was hidden. Clear it so L.map
      // doesn't throw "Map container is already initialized."
      const leafletNode = node as HTMLDivElement & { _leaflet_id?: number };
      if (leafletNode._leaflet_id != null) {
        leafletNode._leaflet_id = undefined;
        leafletNode.replaceChildren();
      }

      const map = new LeafletMap(node, options);
      mapInstanceRef.current = map;
      map.setView(center, zoom);
      if (whenReady != null) {
        map.whenReady(whenReady);
      }
      setContext(createLeafletContext(map));
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [destroyMap],
  );

  useEffect(() => {
    return () => {
      destroyMap();
    };
  }, [destroyMap]);

  return (
    <div {...props} ref={mapRef}>
      {context ? (
        <LeafletProvider value={context}>{children}</LeafletProvider>
      ) : null}
    </div>
  );
}

const SafeMapContainer = forwardRef(SafeMapContainerComponent);
export default SafeMapContainer;
