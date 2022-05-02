let
	pkgs = (import <nixos> {});
	python3WPkgs = pkgs.python3.withPackages (
		pkgs: with pkgs; [crc16 paho-mqtt iso8601]
	);
in pkgs.stdenv.mkDerivation {
	name = "habitat_uploader_env";
	buildInputs = [python3WPkgs];
}
