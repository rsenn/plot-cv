#!/bin/sh

for ARG; do 
  svgo --indent=2 --pretty \
    --enable={cleanupAttrs,cleanupEnableBackground,cleanupListOfValues,cleanupNumericValues,convertColors,convertShapeToPath,convertTransform,inlineStyles,mergePaths,moveGroupAttrsToElems,prefixIds,removeComments,removeDesc,removeDoctype,removeEditorsNSData,removeEmptyAttrs,removeEmptyContainers,removeEmptyText,removeHiddenElems,removeMetadata,removeNonInheritableGroupAttrs,removeOffCanvasPaths,removeRasterImages,removeScriptElement,removeTitle,removeUnknownsAndDefaults,removeUnusedNS,removeUselessDefs,removeUselessStrokeAndFill,removeXMLProcInst,reusePaths,convertPathData,sortAttrs} \
    -i "$ARG" -o "${ARG##*/}"

  sed -i 's,xlink:,,g' "${ARG##*/}" 
done
