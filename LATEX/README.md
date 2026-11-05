# Informe técnico AiGreenBots

- `main.tex`: formato, portada y metadatos, adaptados de `NELSON/02_AiGreenBots/LATEX/REPORTE_TECNICO/main.tex`.
- `contenido.tex`: desarrollo formal, ecuaciones, procedimiento y referencias a implementación.
- `imagenes/`: logotipos de la plantilla.
- `main.pdf`: documento compilado.
- `historico/main_borrador_inicial.tex`: versión anterior conservada.
- `bibliografia.bib`: bibliografía heredada; el documento actual utiliza referencias de implementación mediante `thebibliography`.

Compilación desde esta carpeta:

```bash
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

El informe describe el código revisado el 4 de noviembre de 2026. No se han ejecutado nuevas campañas de robot para producir resultados. Antes de añadir cifras deben adjuntarse configuración, revisión del código, registros, protocolo y cálculo de métricas. Los estados actuales se verifican en `conceptos.md`, TOML y fuentes C++, pues el README general conserva algunas descripciones históricas.

La portada conserva el diseño del proyecto sin atribuir este borrador automáticamente a un entregable oficial ni su aprobación a los revisores de la plantilla.
