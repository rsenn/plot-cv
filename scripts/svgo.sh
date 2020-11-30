#!/bin/sh

for ARG; do 
  svgo --indent=2 --pretty --disable=mergePaths \
    --enable={cleanupAttrs,cleanupEnableBackground,cleanupListOfValues,cleanupNumericValues,convertColors,convertShapeToPath,convertTransform,convertPathData,inlineStyles,moveGroupAttrsToElems,prefixIds,removeComments,removeDesc,removeDoctype,removeEditorsNSData,removeEmptyAttrs,removeEmptyContainers,removeEmptyText,removeHiddenElems,removeMetadata,removeNonInheritableGroupAttrs,removeOffCanvasPaths,removeRasterImages,removeScriptElement,removeTitle,removeUnknownsAndDefaults,removeUnusedNS,removeUselessDefs,removeUselessStrokeAndFill,removeXMLProcInst,convertPathData,sortAttrs} \
    -i "$ARG" -o "${ARG##*/}"

  sed -i 's,xlink:,,g' "${ARG##*/}" 
done
