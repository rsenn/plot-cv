#!/bin/sh

for ARG; do 
  svgo --indent=2 --enable=removeComments,removeDesc,removeEditorsNSData,removeMetadata,convertStyleToAttrs --pretty \
    -i "$ARG" -o "${ARG##*/}"

  sed -i 's,xlink:,,g' "${ARG##*/}" 
done
    #--enable={moveGroupAttrsToElems,prefixIds,removeComments,removeDesc,removeDoctype,removeEditorsNSData,removeEmptyAttrs,removeEmptyContainers,removeEmptyText,removeHiddenElems,removeMetadata,removeNonInheritableGroupAttrs,removeOffCanvasPaths,removeRasterImages,removeScriptElement,removeTitle,removeUnknownsAndDefaults,removeUnusedNS,removeUselessDefs,removeUselessStrokeAndFill,removeXMLProcInst,convertPathData,sortAttrs} \
