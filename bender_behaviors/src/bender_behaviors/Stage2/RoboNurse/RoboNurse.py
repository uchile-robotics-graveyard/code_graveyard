# Resumen de los userdata que se me ocurrieron utiles:
# contador      --->        int         ---> contador para saber cuantos ha hecho bender en un solo trabajo
# askedPills   --->         Pills       ---> Objeto que deberia tener una descripcion de las pastillas
# detectedPills --->      list(Object)  ---> Para saber que Pastillas a detectado.  El objeto deberia tener coordenadas y descripción
# choice        --->        int         --->    un 0 o 1, se usa en MoveToPatient


# # # # # # # # # # # # # # #
# # # STATES DEFINITION # # #
# # # # # # # # # # # # # # #


class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted']
        )

    def execute (self , userdata):

        # SETUP
        return 'succeeded'
        

class WaitForCall(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['failed','succeeded','aborted','preempted'],
            output_keys = ['option']
        )

    def execute(self , userdata):

        # Esperar para la llamada para comenzar
        userdata.option = 0
        return 'succeeded'


class AskForCall(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted']
        )

    def execute(self , userdata):

        # Pedir que haga un llamado ya que no detecto ningun tipo de llamado
        return 'succeeded'


class MoveToPatient(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['continueListen','continueHandover','succeeded','aborted','preempted'],
            input_keys = ['option']
        )

    def execute(self , userdata):

        # Moverse hacia donde está el paciente

        if userdata.option == 0:
            return 'continueListen'
        elif userdata.option == 1:
            return 'continueHandover'

class ListenPills(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['failed','succeded','aborted','preempted'],
            input_keys = ['contador'],
            output_keys = ['contador','askedPills']
        )

    def execute(self , userdata):

        if userdata.contador == 0:
            # Preguntar que pastillas quiere
            pass
        elif userdata.contador > 0:
            # Pedir que repita que pastillas quiere
            pass


        # TODO:
        pass
        userdata.askedPills = "asd"# Esperar a escuchar algo y guardarlo en esta variable
        if userdata.askedPills != "sadasd":# el valor inicial que uno le puede poner a askedPills

            userdata.contador = 0
            return 'succeeded'

        else:
            if userdata.contador == 3:
                return 'preempted'
            else:
                userdata.contador += 1
                return 'failed'

class MoveToShelf(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted']
        )

    def execute(self , userdata):

        # Moverse al velador
        return 'succeeded'

class FindPills(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['failed','succeeded','aborted','preempted'],
            input_keys = ['contador','detectedPills'],
            output_keys = ['contador']
        )

    def execute(self , userdata):

        pass
        # detectar de izquierda a derecha o de centro hacia afuera o como sea.
        #TODO
        if True:# detecta una pastilla cualquiera
            userdata.detectedPills += "sadsa" # Objeto Pastilla que tiene las coordenadas y la descripción.
            userdata.contador = 0
            return 'succeeded'

        else:
            if contador == 3:
                return 'preempted'
            else:
                userdata.contador += 1
                return 'failed'


class DescribePills(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['failed','succeeded','aborted','preempted'],
            input_keys = ['detectedPills','askedPills']
        )

    def execute(self , userdata):

        pass
        #TODO
        # Describe a la pastilla detectada en voz alta
        if True:# La descripción de la ultima pastilla calza con la de userdata.askedPills
            return 'succeeded'

        else:
            return 'failed'

class GraspPills(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['failed','succeeded','aborted','preempted'],
            input_keys = ['contador','detectedPills'],
            output_keys = ['option']
        )

    def execute(self , userdata):

        pass
        # TODO
        if True:# agarra la ultima pastilla detectada
            userdata.contador = 0
            userdata.option = 1
            return 'succeeded'

        else:
            if userdata.contador == 3:
                return 'preempted'
            else:
                userdata.contador += 1
                return 'failed'


class  HandoverPills(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted']
        )

    def __init__(self , userdata):

        # Entregar las pastillas
        return 'succeeded'




# # # # # # # # # # # # #
# # # STATE MACHINE # # #
# # # # # # # # # # # # #


def getmachine ():

    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    rospack = rospkg.RosPack()

    userdata.detectedPills = []
    userdata.contador = 0
    userdata.option = 0


    with sm:

        smach.StateMachine.add('SETUP', Setup(),
            transitions  =  {'succeeded':'WAIT_FOR_CALL'}
        )

        smach.StateMachine.add ('WAIT_FOR_CALL', WaitForCall(),
            transitions  =  {'succeeded':'MOVE_TO_PATIENT'}
        )

        smach.StateMachine.add ('MOVE_TO_PATIENT', MoveToPatient(),
            transitions  =  {'continueListen':'LISTEN_PILLS',
                             'continueHandover':'HANDOVER_PILLS'}
        )

        smach.StateMachine.add ('LISTEN_PILLS',ListenPills(),
            transitions  =  {'succeeded':'MOVE_TO_SHELF',
                             'failed':'LISTEN_PILLS',
                             'preempted':'preempted'},
            remapping   =   {'contador':'contador', 
                             'askedPills':'askedPills'}
        )

        smach.StateMachine.add ('MOVE_TO_SHELF', MoveToShelf(),
            transitions  =  {'succeeded':'FIND_PILLS'}
        )

        smach.StateMachine.add ('FIND_PILLS',FindPills(),
            transitions  =  {'succeeded':'DESCRIBE_PILLS',
                             'failed':'FIND_PILLS',
                             'preempted':'preempted'},
            remapping   =   {'contador':'contador',
                             'detectedPills':'detectedPills'}
        )

        smach.StateMachine.add ('DESCRIBE_PILLS', DescribePills(),
            transitions  =  {'succeeded':'GRASP_PILLS'}
        )

        smach.StateMachine.add ('GRASP_PILLS',GraspPills(),
            transitions  =  {'succeeded':'MOVE_TO_PATIENT',
                             'failed':'GRASP_PILLS',
                             'preempted':'preempted'},
            remapping   =   {'contador':'contador',
                             'option':'option'}
        )

        smach.StateMachine.add ('HANDOVER_PILLS',HandoverPills(),
            transitions  =  {'succeeded':'succeeded'}
        )





if __name__ == '__main__':

    rospy.init_node('roboNurse')

    sm = getMachine()

    sis = smach_ros.IntrospectionServer('roboNurse', sm, '/ROBONURSE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
