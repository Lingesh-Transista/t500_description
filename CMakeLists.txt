install(
  DIRECTORY urdf meshes worlds launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)
