"""Compara el coste de la percepción de la fase 1 (máscara de la franja + momentos)
con la de la fase 2 (scanlines + ajustes). Se ejecuta en el contenedor:
docker exec -w /tmp/sjk robotics python3 tools/bench_percepcion.py"""
import sys
import timeit

sys.path[:0] = ["src", "tests"]
import follow_line as fl  # noqa: E402
import test_percepcion as t  # noqa: E402

casos = {"recta": t.pista_sintetica(), "curva R=8": t.pista_sintetica(radio=8.0)}
N = 2000
print(f"{'caso':<11}{'fase 1 (ms)':>13}{'fase 2 (ms)':>13}{'solo filas (ms)':>17}")
for nombre, img in casos.items():
    a = timeit.timeit(lambda: fl.centro_linea(img), number=N) / N * 1000
    b = timeit.timeit(lambda: fl.percepcion(img), number=N) / N * 1000
    c = timeit.timeit(lambda: fl.centros_filas(img), number=N) / N * 1000
    print(f"{nombre:<11}{a:>13.3f}{b:>13.3f}{c:>17.3f}")
