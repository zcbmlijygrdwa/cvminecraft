file(REMOVE_RECURSE
  "tool.pdb"
  "tool"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/tool.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
