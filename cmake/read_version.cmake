# Reads the content of the `version` tag in the package.xml file
function (read_version OUT_VERSION)
    # read the package.xml file
    file(READ ${CMAKE_CURRENT_LIST_DIR}/package.xml PACKAGE_XML_CONTENTS)

    # find the version tag
    string(REGEX MATCH "^.*<version>([^<]*)</version>.*$" match ${PACKAGE_XML_CONTENTS})
    if(match)
        # return the text of the version tag
        set(${OUT_VERSION} ${CMAKE_MATCH_1} PARENT_SCOPE)
    endif()
endfunction()