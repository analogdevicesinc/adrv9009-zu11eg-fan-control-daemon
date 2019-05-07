option(WITH_SYSTEMD "Enable installation of systemd service file for fancontrold" OFF)
set(SYSTEMD_UNIT_INSTALL_DIR /lib/systemd/system CACHE PATH "default install path for systemd unit files")
option(WITH_SYSVINIT "Enable installation of SysVinit script for fancontrold" OFF)
set(SYSVINIT_INSTALL_DIR /etc/init.d CACHE PATH "default install path for SysVinit scripts")
option(WITH_UPSTART "Enable installation of upstart config file for fancontrold" OFF)
set(UPSTART_CONF_INSTALL_DIR /etc/init CACHE PATH "default install path for upstart conf files")

if(WITH_SYSVINIT)
	configure_file(fancontrold.init.cmakein
			${CMAKE_SOURCE_DIR}/init/fancontrold
			@ONLY)

	install(FILES ${CMAKE_SOURCE_DIR}/init/fancontrold
		PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
		DESTINATION ${SYSVINIT_INSTALL_DIR})

	install(CODE "execute_process(COMMAND update-rc.d fancontrold defaults 99 1)")
endif()

if(WITH_SYSTEMD)
	configure_file(fancontrold.service.cmakein
			${CMAKE_SOURCE_DIR}/init/fancontrold.service
			@ONLY)

	install(FILES ${CMAKE_SOURCE_DIR}/init/fancontrold.service
		DESTINATION ${SYSTEMD_UNIT_INSTALL_DIR})
endif()

if(WITH_UPSTART)
	configure_file(fancontrold.conf.cmakein
			${CMAKE_SOURCE_DIR}/init/fancontrold.conf
			@ONLY)

	install(FILES ${CMAKE_SOURCE_DIR}/init/fancontrold.conf
		DESTINATION ${UPSTART_CONF_INSTALL_DIR})
endif()
