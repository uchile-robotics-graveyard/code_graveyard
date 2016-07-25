#!/bin/bash

set -o errexit

COMMAND_NAME="benderlaunch"
SCRIPT_FILE_NAME=$0
CONFIG_FILE_NAME="benderlaunch.cfg"
DEFAULT_EDITOR="gedit"
LAUNCH_LIST=""

function _error
{
	echo "[ERROR] $@" 1>&2
}

function _warn
{
	echo "[WARN] $@" 1>&2
}

function _info
{
	echo "[INFO] $@"
}

function get_script_dir
{
	local source="${BASH_SOURCE[0]}"
	local dir=""
	while [ -h "$source" ]
	do
		dir=$(cd -P "$(dirname "$source")" && pwd)
		source=$(readlink "$source")
		[[ $source != /* ]] && source="$dir/$source"
	done
	cd -P "$(dirname "$source")" && pwd
}

function resolve_envvars
{
	local input_string=$1
	local parent_file=$2

	while [ 0 ]
	do
		# Verificar que existan variables por resolver
		local envvar_name=$(									\
			echo $input_string			 						\
			| grep -oE '\$\(\s*env\s+\S+\s*\)'					\
			| sed -r 's/\$\(\s*env\s+(\S+)\s*\)/\1/'			\
		)

		[ "$envvar_name" = "" ] && break

		# No se si esto es potencialmente inseguro, 'echo' debiera
		# tener prioridad antes de evaluar otro comando ...
		local envvar_value=$(eval echo '$'"$envvar_name")

		if [ "$envvar_value" = "" ]
		then
			_warn "Variable '$envvar_name' no encontrada."		\
				  "Verifique el archivo de configuracion"		\
				  "(archivo = $parent_file)"
			break
		fi

		input_string=$(											\
			echo $input_string									\
			| sed -r "s/\\$\(\s*env\s+$envvar_name\s*\)/$envvar_value/"\
		)
	done

	echo "$input_string"
}

function resolve_packages
{
	local input_string=$1
	local parent_file=$2

	while [ 0 ]
	do
		# Verificar que existan variables por resolver
		local package_name=$(									\
			echo $input_string			 						\
			| grep -oE '\$\(\s*find\s+\S+\s*\)'					\
			| sed -r 's/\$\(\s*find\s+(\S+)\s*\)/\1/'			\
		)

		[ "$package_name" = "" ] && break

		local package_dir=$(									\
			roscd $package_name &>/dev/null						\
			&& pwd | sed -r 's/( |\/)/\\\1/g'					\
		)

		if [ "$package_dir" = "" ]
		then
			_warn "Paquete '$package_name' no encontrado."		\
				  "Verifique el archivo de configuracion"		\
				  "(archivo = $parent_file)"
			break
		fi

		input_string=$(											\
			echo $input_string									\
			| sed -r "s/\\$\(\s*find\s+$package_name\s*\)/$package_dir/"\
		)
	done

	echo "$input_string"
}

function resolve_variable
{
	local var_name=$1
	local parent_file=$2

	# Buscar variable en archivo actual
	local variable=$(											\
		grep -E "<arg\s+(.*\s)?name=\"$var_name\"" "$parent_file"\
		| grep -E '\sdefault="[^"]+"(\s|/>)'					\
		| sed -r 's/.*\sdefault="([^"]+)"(\s|\/>).*/\1/'		\
	)

	[ "$variable" != "" ] && echo "$variable" && return

	# Crear lista de archivos incluidos y empezar desde el ultimo
	included_files=$(											\
		grep -E '<include\s+(.*\s)?file="[^"]+"' "$parent_file"	\
		| sed -r 's/.*\sfile="([^"]+)"(\s|\/>).*/\1/'			\
		| tac													\
	)

	# Si no lo encuentra, buscar en includes anteriores
	while read include_file
	do
		# Resolver algunas variables en la ruta del archivo incluido.
		# Notar que no se resuelven variables 'arg' para no entrar en
		# un loop infinito.
		include_file=$(resolve_envvars "$include_file" "$parent_file")
		include_file=$(resolve_packages "$include_file" "$parent_file")

		variable=$(resolve_variable "$var_name" "$include_file")

		[ "$variable" != "" ] && echo "$variable" && return
	done < <(echo "$included_files")
}

function resolve_args
{
	local input_string=$1
	local parent_file=$2

	while [ 0 ]
	do
		# Verificar que existan variables por resolver
		local variable_name=$(									\
			echo $input_string			 						\
			| grep -oE '\$\(\s*arg\s+\S+\s*\)'					\
			| sed -r 's/\$\(\s*arg\s+(\S+)\s*\)/\1/'			\
		)

		[ "$variable_name" = "" ] && break

		# Buscar variable recursivamente
		local variable_value=$(									\
			resolve_variable "$variable_name" "$parent_file"	\
		)

		if [ "$variable_value" = "" ]
		then
			_warn "Variable '$variable_name' no definida."		\
				  "Verifique la configuracion del archivo '"	\
				  "$parent_file'."
			break
		fi

		input_string=$(											\
			echo $input_string									\
			| sed -r "s/\\$\(\s*arg\s+$variable_name\s*\)/$variable_value/"\
		)
	done

	echo "$input_string"
}

function check_file_configuration
{
	local machine_name=$1
	local file_path=$2

	_info "Editando $(basename "$file_path") con machine = $machine_name ..."

	# Si el archivo backup no existe, mantener una copia del original
	[ ! -f "$file_path.orig.xml" ] && cp "$file_path" "$file_path.orig.xml"

	# eliminar include para machines
	sed -r 's/(<include(.*)bender.machine\"\/>)//g' \
		-i "$file_path"

	# Quitar tag 'machine' de todos los nodos (y renombrar archivo antiguo)
	sed -r 's/(<node\s+)(.*\s)?machine="[^"]+"( .*)?(\/?>.*)/\1\2\3\4/g'	\
		-i.xml "$file_path"

	# Agregar tag 'machine' con datos correctos
	if [ "$machine_name" != "localhost" ]
	then
		# agregar machines a todos los nodos
		sed -r "s/(<node\s)(.*)/\1machine=\"$machine_name\" \2/g"\
			-i "$file_path"

		# agregar bender.machine a todos los archivos
		MACHINES_TAG='<include file=\"$(env BENDER_CONFIG)\/bender.machine\"\/>'
		sed -r "s/(<launch>)(.*)/\1$MACHINES_TAG\2/g"\
			-i "$file_path"
	fi

	# Obtener lista de includes
	local include_elements=$(												\
		grep -E '<include( .+| +)file=".+\.launch"( .*)?/?>' "$file_path" 	\
		| sed -r 's/.*\sfile="([^"]+)".*/\1/g'								\
	)

	# Despejar argumentos para obtener el path absoluto
	while read launch_file
	do
		[ "$launch_file" = "" ] && break

		# Reemplazar $(env variable)
		launch_file=$(resolve_envvars "$launch_file" "$file_path")

		# echo 1 "$launch_file"

		# Reemplazar $(find package)
		launch_file=$(resolve_packages "$launch_file" "$file_path")

		# echo 2 "$launch_file"

		# Reemplazar $(arg variable)
		launch_file=$(resolve_args "$launch_file" "$file_path")

		# echo 3 "$launch_file"

		if [ ! -f "$launch_file" ]
		then
			_warn "Archivo '$launch_file' no encontrado. Verifique"	\
				  "la configuracion del archivo '$file_path'."
			continue
		fi

		# Ignorar include si ya esta en la lista de archivos a modificar.
		(echo "$LAUNCH_LIST" | grep -qE ";$launch_file$") && continue

		# Modificar archivo recursivamente
		check_file_configuration "$machine_name" "$launch_file"
	done < <(echo "$include_elements")
}

function display_help
{
	cat <<- EOF
	
	USO: $COMMAND_NAME [opciones]
	
	Este script actualiza la configuracion de los launch files
	definidos en el archivo de configuracion $CONFIG_FILE_NAME
	agregando o modificando el tag 'machine' a los nodos que 
	correspondan.

	Si un launch file incluye a otros archivos, estos tambien seran
	modificados con el mismo 'machine' excepto en el caso que ya
	hayan sido marcados para modificar dentro de $CONFIG_FILE_NAME.

	OPCIONES:
	    [vacio]         Actualiza los launch files (Modo default).
	    -e --edit       (ya no se usa) Abre el archivo de configuracion
	                    con el editor de texto configurado en ROS. Por
	                    defecto se usa $DEFAULT_EDITOR.
	    -r --restore    Restaura los launch files a su version
	                    anterior, renombrando los archivos .launch.xml
	    -s --silent     Modo silencioso. No se muestran mensajes.
	                    (falta implementar)
	    -v --verbose    Modo verbose.
	                    (falta implementar)
	    -h --help       Muestra esta ayuda.
	
	EOF

	echo $MACHINES
}

function launch_text_editor
{
	local target="$(get_script_dir)/$CONFIG_FILE_NAME"
	local editor=$([[ -z $EDITOR ]]	\
		&& echo $DEFAULT_EDITOR 	\
		|| echo $EDITOR				\
	)

	echo "opening file: " . $target
	$editor "$target"
}

function restore_launch_files
{
	# REVISA ESTO \/
	echo "Borrando archivos de trabajo ..."
	find "$BENDER_WORKSPACE" -name '*.launch.xml' \
		-exec rm -v '{}' \;
	echo "Restaurando archivos originales ..."
	# find encuentra todos los '*.launch.xml' dentro de $BENDER_WORKSPACE
	# y ejecuta el comando rename con el nombre de cada archivo encontrado
	find "$BENDER_WORKSPACE" -name '*.launch.orig.xml' \
		-exec rename -fv 's/\.launch\.orig\.xml$/\.launch/' '{}' \;
}

function make_launch_list
{
	for machine in "${!launchfiles[@]}"
	do
		while read line
		do
			[[ $line = "" ]] && continue

			local package_name=$(echo $line | awk '{print $1}')
			local package_dir=$(roscd $package_name &>/dev/null && pwd)

			if [ "$package_dir" = "" ]
			then
				_warn "Paquete '$package_name' no encontrado."	\
					  "Verifique el archivo de configuracion"	\
					  "(machine = $machine)"
				continue
			fi

			local launch_name=$(echo $line | awk '{print $2}')
			local launch_dir=$(find $package_dir -name "$launch_name" | head -1)

			if [ "$launch_dir" = "" ]
			then
				_warn "Launcher '$launch_name' no encontrado."	\
					  "Verifique el archivo de configuracion"	\
					  "(machine = $machine, package = $package_name)"
				continue
			fi

			echo "$machine;$launch_dir"
		done < <(echo "${launchfiles[$machine]}")
	done
}

function apply_configuration
{
	# antes de todo, ejecutar un restore, para evitar
	# sobreescribir archivos originales con los ya modificados!
	# restore_launch_files

	_info "Creando lista de archivos a modificar ..."
	LAUNCH_LIST=$(make_launch_list)

	_info "Actualizando configuracion de launch files ..."
	while read line
	do
		local machine=$(echo $line | cut -d';' -f1)
		local launch_dir=$(echo $line | cut -d';' -f2)

		check_file_configuration "$machine" "$launch_dir"
	done < <(echo "$LAUNCH_LIST")

	_info "Configuracion terminada."
}

function main
{
	local ros_cfg_full_path="$BENDER_CONFIG/setup.bash"

	if [[ -z "$BENDER_WORKSPACE" ]]
	then
		_error "La variable global 'BENDER_WORKSPACE' no esta definida."
		exit 1
	elif [[ ! -f "$ros_cfg_full_path" ]]
	then
		_error "No se encuentra el archivo \"$ros_cfg_full_path\""
		exit 1
	fi

	#source "$ros_cfg_full_path"
	
	case $1 in
		-h|--help)
			display_help
			exit
			;;
		-e|--edit)
			launch_text_editor
			;;
		-r|--restore)
			restore_launch_files
			;;
		*)

			local cfg_full_path="$1"

			if [[ ! -f "$cfg_full_path" ]]
			then
				_error "Falta el archivo de configuracion $cfg_full_path"
				exit 1
			fi
			source "$cfg_full_path"

			apply_configuration
			;;
	esac
}

main $@
