if(WITH_PCL)
  find_package(PCL "${MIN_VER_PCL}")
  if(PCL_FOUND)
    set(HAVE_PCL TRUE)
  endif()
endif()
