package edu.miami.cs.home_ontology_app;

import com.github.owlcs.ontapi.OntManagers;
import com.github.owlcs.ontapi.OntologyManager;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLOntologyManager;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import org.junit.Test;

public class TestHomeOntologyApp
{
    private OntologyManager manager = OntManagers.createONT();

    private String validIRIString = "https://protege.stanford.edu/ontologies/pizza/pizza.owl";
    private String invalidIRIString = "https://protege.stanford.edu/ontologies/dummy/dummy.owl";

    private IRI validIRI = IRI.create(validIRIString);
    private IRI invalidIRI = IRI.create(invalidIRIString);

    @Test
    // NOTE: Tests may fail if we do not have a stable internet connection.
    public void testGetOntologyIsNotNullOnValidIRI()
    {
        assertNotNull(HomeOntologyApp.getOntology(manager, validIRI));
    }

    @Test
    public void testGetOntologyIsNullOnInvalidIRI()
    {
        assertNull(HomeOntologyApp.getOntology(manager, invalidIRI));
    }
}