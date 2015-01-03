FILE(REMOVE_RECURSE
  "CMakeFiles/test1.dir/src/test1.o"
  "../bin/test1.pdb"
  "../bin/test1"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/test1.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
