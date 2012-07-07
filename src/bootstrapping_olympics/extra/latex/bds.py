from latex_gen import latex_fragment
import sys
import os
import cPickle
from reprep.node import Node
from StringIO import StringIO
import scipy.io


def bds_learn_reportA(id_set, agent, robot):

    report = load_report(id_set=id_set,
                         agent=agent, robot=robot, phase='learn')

    sys.stderr.write(report.format_tree())

    T = report['estimator/tensors/T/value'].raw_data
    ncmd = T.shape[2]
    
    assert isinstance(report, Node)
    
    def boxed(el, data_node, gid):
        with el.tightbox() as box:
            box.graphics_data(data_node.raw_data, data_node.mime,
                                      width=tsize, gid=gid)

    graphics_path = 'snippets/figures'
    with latex_fragment(sys.stdout, graphics_path=graphics_path) as frag:
        caption = 'Report for robot %r, agent %r. ' % (robot, agent)
        label = 'fig:%s-%s-%s-learn-rA' % (id_set, robot, agent)
        
        with frag.figure(caption=caption, label=label, placement="p") as fig:
            tsize = '3cm'
                 
            fig.hfill()
            with fig.subfigure(caption="Vehicle",
                               label='%s-%s' % (label, 'vehicle')) as sub:
        
                vehicle = report['vehicle/body_data_compact']
                sub.graphics_data(vehicle.raw_data, vehicle.mime,
                                      width=tsize,
                            gid='%s-%s-%s-body_data_compact' % (id_set, robot, agent))
                
            fig.hfill()
            with fig.subfigure(caption="$\Tcove^{sv}$",
                               label='%s-%s' % (label, 'P'))  as sub:
                boxed(sub, report['estimator/tensors/P/value/posneg'],
                       '%s-%s-%s-P' % (id_set, robot, agent))

            fig.hfill()
            with fig.subfigure(caption="$\Tcove^{-1}_{sv}$",
                               label='%s-%s' % (label, 'Pinv'))  as sub:

                boxed(sub, report['estimator/tensors/P_inv_cond/value/posneg'],
                      '%s-%s-%s-P_inv_cond' % (id_set, robot, agent))

#sub.rule(width=tsize, height=tsize, color='gray')
            fig.hfill()
            
            fig.parbreak()
            
            fig.hfill()

            for i in range(ncmd):
                subcaption = '$\TTe^{s\,v\,%d}$' % i
                sublabel = '%s-T%d' % (label, i)
                
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    boxed(sub, report['estimator/tensors/T/slices/%d/value/posneg' % i],
                          '%s-%s-%s-T%d' % (id_set, robot, agent, i))

                fig.hfill()
                
            fig.parbreak()

            fig.hfill()
            for i in range(ncmd):
                subcaption = '$\TUe^{s\,%d}$' % i
                sublabel = '%s-U%d' % (label, i)
                Ui_im = report['estimator/tensors/U/slices/%d/figure1/plot' % i]
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    gid = '%s-%s-%s-U%d' % (id_set, robot, agent, i)
                    sub.graphics_data(Ui_im.raw_data, Ui_im.mime,
                                      width=tsize, gid=gid)
                fig.hfill()
                
            fig.parbreak()

            fig.hfill()
            for i in range(ncmd):
                subcaption = '$\TMe^s_{v\,%d}$' % i
                sublabel = '%s-M%d' % (label, i)
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    boxed(sub, report['estimator/model/M/slices/%d/value/posneg' % i],
                          '%s-%s-%s-M%d' % (id_set, robot, agent, i))

                fig.hfill()
#                    sub.rule(width=tsize, height=tsize, color='gray')

            fig.parbreak()
            fig.hfill()

            for i in range(ncmd):
                subcaption = '$\TNe^{s\,%d}$' % i
                sublabel = '%s-N%d' % (label, i)
                Ui_im = report['estimator/model/N/slices/%d/figure1/plot' % i]
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    gid = '%s-%s-%s-N%d' % (id_set, robot, agent, i)
                    sub.graphics_data(Ui_im.raw_data, Ui_im.mime,
                                      width=tsize, gid=gid)
                fig.hfill()
            
            if False:
                data = {}
                data['T'] = report['estimator/tensors/T/value'].raw_data
                data['U'] = report['estimator/tensors/U/value'].raw_data
                data['M'] = report['estimator/model/M/value'].raw_data
                data['N'] = report['estimator/model/N/value'].raw_data
                data['P'] = report['estimator/tensors/P/value'].raw_data
                data['P_inv_cond'] = report['estimator/tensors/P_inv_cond/value/posneg'].raw_data
                mat = StringIO()
                scipy.io.savemat(mat, data, oned_as='row')
                matdata = mat.getvalue()
                
                fig.parbreak()
                fig.textattachfile('%s-%s-%s-learn.mat' % (id_set, robot, agent),
                                   matdata, 'Matlab format')
       

def load_report(id_set, agent, robot, phase):
    setdir = '/Users/andrea/Downloads/sets'
#    /Users/andrea/Downloads/sets/bv1bds1/reports/Ugn1Se0Vrb1co-bdse1-learn.pickle

    filename = '%s/%s/reports/%s-%s-%s.pickle' % (setdir, id_set, robot, agent, phase)
    if not os.path.exists(filename):
        msg = 'Report %s not found' % filename
        raise Exception(msg)

    with open(filename, 'rb') as f:
        return cPickle.load(f)
        
