build_dir := justfile_directory()
vs_dir := env_var('VINTAGE_STORY')

build cfg='Release':
  dotnet build -c {{cfg}}

run cfg='Release': (build cfg)
  cd {{vs_dir}} && dotnet Vintagestory.dll --tracelog --addModPath {{build_dir}}/bin/{{cfg}}

clean:
  rm -rf bin
  rm -rf obj
