import requests

OSRM_URL = "https://router.project-osrm.org"

def osrm_table(coords):
    coord_str = ";".join([f"{c[1]},{c[0]}" for c in coords])
    url = f"{OSRM_URL}/table/v1/driving/{coord_str}"
    r = requests.get(url); r.raise_for_status()
    return r.json()["durations"]

def osrm_trip(start, stops):
    coords = [start] + stops
    coord_str = ";".join([f"{c[1]},{c[0]}" for c in coords])
    url = f"{OSRM_URL}/trip/v1/driving/{coord_str}?source=first&roundtrip=false&overview=full&geometries=geojson"
    r = requests.get(url); r.raise_for_status()
    return r.json()
