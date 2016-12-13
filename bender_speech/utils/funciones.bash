#!/bin/bash

error_()
{
	local bldred='\e[1;31m' # Red - Bold
	local txtwht='\e[0;37m' # White - Normal

	echo -e "${bldred}Error: $txtwht${@}" 1>&2
	exit 1
}

warn_()
{
	local bldylw='\e[1;33m' # Yellow - Bold
	local txtwht='\e[0;37m' # White - Normal

	echo -e "${bldylw}Warning: $txtwht${@}" 1>&2
}

info_()
{
	local txtwht='\e[0;37m' # White - Normal

	echo -e "$txtwht${@}"
}

info_bold_()
{
	local bldwht='\e[1;37m' # White - Bold
	local txtwht='\e[0;37m' # White - Normal

	echo -e "${bldwht}${@}${txtwht}"
}

prompt_simple()
{
	[ "$auto_mode" ] && return 0

	# Retorna 0 si la respuesta es 'S' o 's'
	# Retorna 1 en caso contrario

	local bldwht='\e[1;37m' # White - Bold
	local txtwht='\e[0;37m' # White - Normal

	echo -e "$bldwht$1 (s/N) $txtwht\c"
	read -n 1 -r && echo

	[[ $REPLY =~ ^[Ss]$ ]] && return 0 || return 1
}

prompt_options()
{
	# Elegir entre valores predeterminados
	# $1  - Mensaje del prompt
	# $2  - Valor predeterminado
	# $3+ - Opciones 

	[ "$auto_mode" ] && REPLY="$2" && info_ "$REPLY" && return

	local bldwht='\e[1;37m' # White - Bold
	local txtwht='\e[0;37m' # White - Normal

	_true="1"
	while [ "$_true" ]
	do
		echo -e "$bldwht$1 ($2) $txtwht\c"
		read -r
		
		[ "$REPLY" = "" ] && REPLY="$2" && break

		(contains_element "$REPLY" "${@:3}") && break

		info_ "Opcion invalida. \c"
	done
}

prompt_range()
{
	# Elegir entre un rango de valores
	# $1 - Mensaje del prompt
	# $2 - Valor predeterminado
	# $3 - Valor minimo
	# $4 - Valor maximo

	[ "$auto_mode" ] && REPLY="$2" && info_ "$REPLY" && return

	local bldwht='\e[1;37m' # White - Bold
	local txtwht='\e[0;37m' # White - Normal

	_true="1"
	while [ "$_true" ]
	do
		echo -e "$bldwht$1 ($2) $txtwht\c"
		read -r
		
		[ "$REPLY" = "" ] && REPLY="$2" && break

		(is_numeric "$REPLY")							\
		&& [ "$(echo "$REPLY" '>=' "$3" | bc)" -eq 1 ]	\
		&& [ "$(echo "$REPLY" '<=' "$4" | bc)" -eq 1 ]	\
		&& break

		info_ "Opcion invalida. \c"
	done
}

is_numeric()
{
	res=$(echo "$@" | grep -E '^ *[-]?[0-9]+(\.[0-9]+)? *$')
	[ "$res" = "" ] && return 1 || return 0
}

contains_element()
{
	local e
	for e in "${@:2}"; do [[ "$e" == "$1" ]] && return 0; done
 	return 1
}

