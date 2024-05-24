{
  description = "Flake for gmsh playground";

  inputs.nixpkgs.url = "github:nixos/nixpkgs";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils}:
    let
    in
      flake-utils.lib.eachDefaultSystem (
        system:
          let
            pkgs' = import nixpkgs { inherit system; };
          in
            rec {
              devShell = pkgs'.mkShell {
                # a list of packages to add to the shell environment
                #
                nativeBuildInputs = [ pkgs'.bear pkgs'.clang ];
                buildInputs = with pkgs'; [clang-tools clang gtest ];
                packages = with pkgs'; [ 
                    gmsh 
                    gfortran
                    boost
                    catch2
                    cmake
                    eigen
                ];
                ];

                shellHook = ''
                PATH="${pkgs'.clang-tools}/bin:$PATH"
                '';
                 # propagate all the inputs from the given derivations
                  # this adds all the tools that can build myPackage to the environment
              };
            }
      );
}
